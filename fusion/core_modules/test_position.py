# fusion/core_modules/test_position.py
import math
import unittest

try:
    from .position import PositionDeadReckoningTracker
    from .geo_math import geo_dist
except (ImportError, ValueError):
    from core_modules.position import PositionDeadReckoningTracker
    from core_modules.geo_math import geo_dist


class TestPositionDeadReckoningTracker(unittest.TestCase):

    def setUp(self):
        self.tracker = PositionDeadReckoningTracker()

    def test_gnss_tracking(self):
        """Ověření, že v RTK režimu tracker přesně sleduje GNSS souřadnice."""
        self.tracker.update_gnss(lat=50.0, lon=14.0, hAcc=0.02, pos_type="NARROW_INT", is_rtk=True, ts_mono=100.0)
        self.assertEqual(self.tracker.lat, 50.0)
        self.assertEqual(self.tracker.lon, 14.0)
        self.assertEqual(self.tracker.hAcc, 0.02)
        self.assertEqual(self.tracker.pos_type, "NARROW_INT")
        self.assertFalse(self.tracker.dr_active)

    def test_dr_under_bridge_straight_north(self):
        """Simulace vjezdu pod most (výpadek GNSS) a jízdy na sever v Dead Reckoning."""
        lat0 = 50.0
        lon0 = 14.0
        # 1. Poslední GNSS zpráva před mostem v t = 100 s
        self.tracker.update_gnss(lat=lat0, lon=lon0, hAcc=0.02, pos_type="NARROW_INT", is_rtk=True, ts_mono=100.0)

        # 2. Vjezd pod most v t = 102.1 s (> 2.0 s bez GNSS -> detekce výpadku)
        # Jízda na sever (heading = 0.0°) rychlostí 1000 mm/s (1 m/s) po dobu 5 sekund
        self.tracker.update_odometry(speed_mm_s=1000.0, is_stationary=False, heading_deg=0.0, heading_valid=True, ts_mono=102.1)
        for i in range(1, 6):
            t = 102.1 + float(i)
            self.tracker.update_odometry(speed_mm_s=1000.0, is_stationary=False, heading_deg=0.0, heading_valid=True, ts_mono=t)

        self.assertTrue(self.tracker.dr_active)
        self.assertEqual(self.tracker.pos_type, "DEAD_RECKONING")
        self.assertAlmostEqual(self.tracker.dr_distance_m, 5.0, delta=0.05)

        # Posun na sever o 5 metrů
        expected_lat = lat0 + (5.0 / 111139.0)
        self.assertAlmostEqual(self.tracker.lat, expected_lat, places=6)
        self.assertAlmostEqual(self.tracker.lon, lon0, places=6)

        # Nejistota hAcc musí vzrůst
        self.assertGreater(self.tracker.hAcc, 0.02)

    def test_dr_under_bridge_east(self):
        """Jízda na východ (heading = 90.0°) pod mostem."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat=lat0, lon=lon0, hAcc=0.02, pos_type="NARROW_INT", is_rtk=True, ts_mono=100.0)

        # 10 sekund jízdy na východ rychlostí 1.0 m/s
        self.tracker.update_odometry(1000.0, False, 90.0, True, ts_mono=103.0)
        for i in range(1, 11):
            t = 103.0 + float(i)
            self.tracker.update_odometry(1000.0, False, 90.0, True, ts_mono=t)

        self.assertTrue(self.tracker.dr_active)
        self.assertAlmostEqual(self.tracker.dr_distance_m, 10.0, delta=0.05)

        # Zeměpisná šířka se nezměnila, délka se posunula na východ
        self.assertAlmostEqual(self.tracker.lat, lat0, places=6)
        expected_lon = lon0 + (10.0 / (111139.0 * math.cos(math.radians(lat0))))
        self.assertAlmostEqual(self.tracker.lon, expected_lon, places=6)

    def test_dr_zupt_standstill(self):
        """ZUPT pod mostem: stání robota nezpůsobuje žádný posun ani drift."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat=lat0, lon=lon0, hAcc=0.02, pos_type="NARROW_INT", is_rtk=True, ts_mono=100.0)

        # Robot zastavil pod mostem
        self.tracker.update_odometry(0.0, True, 45.0, True, ts_mono=103.0)
        self.tracker.update_odometry(0.0, True, 45.0, True, ts_mono=105.0)
        self.tracker.update_odometry(0.0, True, 45.0, True, ts_mono=110.0)

        self.assertTrue(self.tracker.dr_active)
        self.assertEqual(self.tracker.dr_distance_m, 0.0)
        self.assertAlmostEqual(self.tracker.lat, lat0, places=7)
        self.assertAlmostEqual(self.tracker.lon, lon0, places=7)

    def test_reacquisition_exit_bridge(self):
        """Výjezd zpod mostu a re-akvizice RTK fixu."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat0, lon0, 0.02, "NARROW_INT", True, ts_mono=100.0)

        # Jízda pod mostem 5 m na sever
        self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=103.0)
        self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=108.0)
        self.assertTrue(self.tracker.dr_active)

        # Výjezd zpod mostu v t = 109 s -> naskakuje nový RTK fix
        exit_lat = lat0 + (5.1 / 111139.0)
        exit_lon = lon0
        self.tracker.update_gnss(exit_lat, exit_lon, 0.02, "NARROW_INT", True, ts_mono=109.0)

        self.assertFalse(self.tracker.dr_active)
        self.assertEqual(self.tracker.pos_type, "NARROW_INT")
        self.assertAlmostEqual(self.tracker.lat, exit_lat, delta=0.1)

    def test_phase1_free_dr_ignores_single_jump(self):
        """Fáze 1 (<50 m): Odraz GNSS o 6 metrů do strany je zcela ignorován."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat0, lon0, 0.02, "NARROW_INT", True, ts_mono=100.0)

        # Ujetí 20 m na sever v Dead Reckoning
        self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=101.0)
        for i in range(1, 21):
            t = 101.0 + float(i)
            # Pod mostem přichází SINGLE s odskokem o 6 metrů na východ
            jump_lon = lon0 + (6.0 / (111139.0 * math.cos(math.radians(lat0))))
            self.tracker.update_gnss(lat0 + (float(i) / 111139.0), jump_lon, 5.0, "SINGLE", False, ts_mono=t)
            self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=t)

        self.assertEqual(self.tracker.fusion_phase, 1)
        self.assertFalse(self.tracker.leash_active)
        # DR souřadnice nesmí být ovlivněny 6metrovým skokem na východ
        self.assertAlmostEqual(self.tracker.lon, lon0, places=6)
        expected_lat = lat0 + (20.0 / 111139.0)
        self.assertAlmostEqual(self.tracker.lat, expected_lat, places=6)

    def test_phase2_leash_clamping_on_slip(self):
        """Fáze 2 (50-120 m): Prokluz kol táhne DR moc daleko, vodítko ho nepustí za r = 1.5 * hAcc."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat0, lon0, 0.02, "NARROW_INT", True, ts_mono=100.0)

        # Ujetí 70 metrů na sever
        self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=101.0)
        for i in range(1, 71):
            t = 101.0 + float(i)
            # GPS SINGLE zůstává poblíž lat0 + 40m, hAcc = 3.0 m (max povolený rádius 1.5 * 3.0 = 4.5 m)
            gps_lat = lat0 + (40.0 / 111139.0)
            self.tracker.update_gnss(gps_lat, lon0, 3.0, "SINGLE", False, ts_mono=t)
            self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=t)

        self.assertEqual(self.tracker.fusion_phase, 2)
        self.assertTrue(self.tracker.leash_active)
        # Vzdálenost DR polohy od GPS nesmí překročit 1.5 * 3.0 = 4.5 m
        dist = geo_dist(self.tracker.lat, self.tracker.lon, gps_lat, lon0)
        self.assertAlmostEqual(dist, 4.5, delta=0.05)

    def test_phase3_long_outage_blend(self):
        """Fáze 3 (>120 m): Dlouhý výpadek zapojuje jemný tah ke středu GNSS."""
        lat0 = 50.0
        lon0 = 14.0
        self.tracker.update_gnss(lat0, lon0, 0.02, "NARROW_INT", True, ts_mono=100.0)

        # Ujetí 130 metrů
        self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=101.0)
        for i in range(1, 131):
            t = 101.0 + float(i)
            gps_lat = lat0 + (100.0 / 111139.0)
            self.tracker.update_gnss(gps_lat, lon0, 4.0, "SINGLE", False, ts_mono=t)
            self.tracker.update_odometry(1000.0, False, 0.0, True, ts_mono=t)

        self.assertEqual(self.tracker.fusion_phase, 3)
        self.assertTrue(self.tracker.leash_active)


if __name__ == '__main__':
    unittest.main()

