# fusion/core_modules/test_heading.py
import unittest

try:
    from .heading import HeadingManager, HeadingData
    from .constellation import AntennaState
    from .geo_math import compute_tangent_offset
except (ImportError, ValueError):
    from core_modules.heading import HeadingManager, HeadingData
    from core_modules.constellation import AntennaState
    from core_modules.geo_math import compute_tangent_offset


class TestHeadingManager(unittest.TestCase):

    def setUp(self):
        self.mgr = HeadingManager()

    def test_uniheading_init_and_blend(self):
        """Ověření inicializace z UNIHEADING a komplementárního vyhlazování (alpha=0.1)."""
        self.mgr.update_dual_heading(heading=90.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.assertTrue(self.mgr.heading_initialized)
        self.assertEqual(self.mgr.heading_source, "UNIHEADING")
        self.assertAlmostEqual(self.mgr.fused_heading.heading, 90.0)

        # Druhá zpráva s odchylkou +10° -> posun o 0.1 * 10 = +1° na 91°
        self.mgr.update_dual_heading(heading=100.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.assertAlmostEqual(self.mgr.fused_heading.heading, 91.0)

        # Ztráta NARROW_INT -> přechod na GYRO
        self.mgr.update_dual_heading(heading=100.0, headingAcc=4.0, headingSol="SINGLE")
        self.assertEqual(self.mgr.heading_source, "GYRO")

    def test_single_course_lock_3_samples(self):
        """Ověření uzamčení kurzu po 3 vzorcích v toleranci +-3° při přímé jízdě."""
        # 1. Stojí -> kurz se nebere
        self.mgr.update_single_course(heading=180.0, headingAcc=2.0, headingSol="FIXED",
                                      hor_spd=0.0, gyroZ=0.0, is_stationary=True)
        self.assertFalse(self.mgr.heading_initialized)
        self.assertEqual(len(self.mgr.bestnav_course_buffer), 0)

        # 2. Jede rovně, 3 vzorky v toleranci +-3°
        self.mgr.update_single_course(180.0, 2.0, "FIXED", 0.45, 0.5, False)
        self.mgr.update_single_course(181.5, 2.0, "FIXED", 0.45, 0.5, False)
        self.mgr.update_single_course(179.0, 2.0, "FIXED", 0.45, 0.5, False)

        self.assertTrue(self.mgr.heading_initialized)
        self.assertEqual(self.mgr.heading_source, "BESTNAV_COURSE")
        self.assertAlmostEqual(self.mgr.fused_heading.heading, 180.17, places=1)

    def test_3_antenna_consensus_in_turn(self):
        """Ověření 3-anténního konsenzu s kinematickou kompenzací tečného úhlu v zatáčce."""
        ant_gps = AntennaState("gnss-gps", offset_x=0.32, offset_y=0.0, valid=True, pos_type="NARROW_INT")
        ant_master = AntennaState("gnss-dual-master", offset_x=0.25, offset_y=0.24, valid=True, pos_type="NARROW_INT")
        ant_slave = AntennaState("gnss-dual-slave", offset_x=0.25, offset_y=-0.24, valid=True, pos_type="NARROW_INT")

        true_heading = 45.0
        wz = 6.0
        v = 0.5

        beta_gps = compute_tangent_offset(0.32, 0.0, v, wz)
        beta_mst = compute_tangent_offset(0.25, 0.24, v, wz)
        beta_slv = compute_tangent_offset(0.25, -0.24, v, wz)

        ant_gps.trk_gnd = true_heading + beta_gps
        ant_gps.hor_spd = v
        ant_gps.ts_mono = 100.0

        ant_master.trk_gnd = true_heading + beta_mst
        ant_master.hor_spd = v
        ant_master.ts_mono = 100.0

        ant_slave.trk_gnd = true_heading + beta_slv
        ant_slave.hor_spd = v
        ant_slave.ts_mono = 100.0

        import time
        now = time.monotonic()
        ant_gps.ts_mono = now
        ant_master.ts_mono = now
        ant_slave.ts_mono = now

        self.mgr.evaluate_consensus(ant_gps, ant_master, ant_slave, speed=v, gyroZ=wz, is_stationary=False)

        self.assertEqual(self.mgr.heading_consensus_status, "3_ANTENNAS_AGREE")
        self.assertEqual(self.mgr.heading_source, "BESTNAV_CONSENSUS")
        self.assertAlmostEqual(self.mgr.fused_heading.heading, true_heading, places=1)

    def test_zupt_gyro_drift_ignored_when_stationary(self):
        """Ověření ignorování driftu gyra v klidu (ZUPT)."""
        self.mgr.update_dual_heading(heading=90.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.mgr.update_dual_heading(heading=90.0, headingAcc=5.0, headingSol="SINGLE")  # přepne na GYRO

        # V klidu přichází drift: wz = 0.3°/s, delta_yaw = 0.05°
        self.mgr.update_imu(delta_yaw=0.05, wz=0.3, is_stationary=True)
        self.assertAlmostEqual(self.mgr.fused_heading.heading, 90.0)

        # Při pohybu se integruje
        self.mgr.update_imu(delta_yaw=10.0, wz=20.0, is_stationary=False)
        self.assertAlmostEqual(self.mgr.fused_heading.heading, 100.0)


if __name__ == '__main__':
    unittest.main()
