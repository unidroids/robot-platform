# fusion/core_modules/test_constellation.py
import math
import unittest

try:
    from .constellation import ConstellationManager
except (ImportError, ValueError):
    from core_modules.constellation import ConstellationManager


class TestConstellationManager(unittest.TestCase):

    def setUp(self):
        self.mgr = ConstellationManager()

    def test_nominal_triangle_and_center_projection(self):
        """Ověření správného vyhodnocení nominální geometrie a transformace do středu otáčení."""
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * math.cos(math.radians(50.0))

        # Antény v nominálních offsetech vzhledem k [lat0, lon0]:
        # GPS: [+0.32, 0.0] m
        gps_lat = lat0 + 0.32 / m_deg_lat
        gps_lon = lon0

        # Master: [+0.25, +0.24] m
        master_lat = lat0 + 0.25 / m_deg_lat
        master_lon = lon0 + 0.24 / m_deg_lon

        # Slave: [+0.25, -0.24] m
        slave_lat = lat0 + 0.25 / m_deg_lat
        slave_lon = lon0 - 0.24 / m_deg_lon

        self.mgr.update_gps(gps_lat, gps_lon, 0.02, "NARROW_INT")
        self.mgr.update_master(master_lat, master_lon, 0.02, "NARROW_INT")
        self.mgr.update_slave(slave_lat, slave_lon, 0.02, "NARROW_INT")

        # Vyhodnocení při kurzu 0.0° (Sever)
        self.mgr.evaluate_constellation_and_position(heading_deg=0.0, heading_initialized=True)

        self.assertEqual(self.mgr.triangle_status, "TRIANGLE_OK")
        self.assertEqual(self.mgr.active_antenna_name, "gnss-gps")
        self.assertAlmostEqual(self.mgr.dist_dual, 0.48, delta=0.02)
        self.assertAlmostEqual(self.mgr.dist_gps_master, 0.25, delta=0.02)
        self.assertAlmostEqual(self.mgr.dist_gps_slave, 0.25, delta=0.02)

        # Střed otáčení musí odpovídat [lat0, lon0]
        self.assertAlmostEqual(self.mgr.center_lat, lat0, places=6)
        self.assertAlmostEqual(self.mgr.center_lon, lon0, places=6)

    def test_gps_multipath_exclusion(self):
        """Ověření vyloučení odlehlé antény GPS (např. multipath) a přepnutí na Master."""
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * math.cos(math.radians(50.0))

        master_lat = lat0 + 0.25 / m_deg_lat
        master_lon = lon0 + 0.24 / m_deg_lon
        slave_lat = lat0 + 0.25 / m_deg_lat
        slave_lon = lon0 - 0.24 / m_deg_lon

        # GPS má chybu 3 metry
        bad_gps_lat = lat0 + 3.32 / m_deg_lat
        bad_gps_lon = lon0

        self.mgr.update_gps(bad_gps_lat, bad_gps_lon, 0.02, "NARROW_INT")
        self.mgr.update_master(master_lat, master_lon, 0.02, "NARROW_INT")
        self.mgr.update_slave(slave_lat, slave_lon, 0.02, "NARROW_INT")

        self.mgr.evaluate_constellation_and_position(heading_deg=0.0, heading_initialized=True)

        self.assertEqual(self.mgr.triangle_status, "EXCLUDED_GPS")
        self.assertEqual(self.mgr.active_antenna_name, "gnss-dual-master")

        # Transformovaná poloha ze záložní Master antény musí odpovídat středu [lat0, lon0]
        self.assertAlmostEqual(self.mgr.center_lat, lat0, places=6)
        self.assertAlmostEqual(self.mgr.center_lon, lon0, places=6)

    def test_diagnostics(self):
        """Ověření diagnostických struktur."""
        self.mgr.update_gps(50.0, 14.0, 0.05, "FIXED")
        c_diag = self.mgr.get_constellation_diagnostics()
        a_diag = self.mgr.get_antennas_diagnostics()

        self.assertIn("status", c_diag)
        self.assertIn("active_antenna", c_diag)
        self.assertIn("gps", a_diag)
        self.assertIn("master", a_diag)
        self.assertIn("slave", a_diag)
    def test_narrow_int_required_for_triangle_evaluation(self):
        """Ověření, že bez NARROW_INT se geometrie konstelace nevyhodnocuje."""
        # 1. Antény mají pouze SINGLE fix (přesnost v metrech)
        self.mgr.update_gps(50.000003, 14.0, 1.5, "SINGLE")
        self.mgr.update_master(50.000002, 14.000003, 1.5, "SINGLE")
        self.mgr.update_slave(50.000002, 13.999997, 1.5, "SINGLE")

        self.mgr.evaluate_constellation_and_position(heading_deg=0.0, heading_initialized=False)
        self.assertEqual(self.mgr.triangle_status, "NO_NARROW_INT")
        self.assertEqual(self.mgr.dist_dual, 0.0)
        self.assertEqual(self.mgr.dist_gps_master, 0.0)
        self.assertEqual(self.mgr.dist_gps_slave, 0.0)
        # Základní poloha je ale pro robota stále dostupná
        self.assertTrue(self.mgr.have_position)
        self.assertEqual(self.mgr.active_antenna_name, "gnss-gps")

        # 2. Jakmile naskočí NARROW_INT (RTK), geometrie se okamžitě spočítá
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * 0.6427876
        self.mgr.update_gps(lat0 + 0.32 / m_deg_lat, lon0, 0.02, "NARROW_INT")
        self.mgr.update_master(lat0 + 0.25 / m_deg_lat, lon0 + 0.24 / m_deg_lon, 0.02, "NARROW_INT")
        self.mgr.update_slave(lat0 + 0.25 / m_deg_lat, lon0 - 0.24 / m_deg_lon, 0.02, "NARROW_INT")

        self.mgr.evaluate_constellation_and_position(heading_deg=0.0, heading_initialized=True)
        self.assertEqual(self.mgr.triangle_status, "TRIANGLE_OK")
        self.assertAlmostEqual(self.mgr.dist_dual, 0.48, delta=0.02)


if __name__ == '__main__':
    unittest.main()
