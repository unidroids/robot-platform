# fusion/core_modules/test_odometry.py
import time
import unittest

try:
    from .odometry import OdometryTracker
except (ImportError, ValueError):
    from core_modules.odometry import OdometryTracker


class TestOdometryTracker(unittest.TestCase):

    def setUp(self):
        self.tracker = OdometryTracker()

    def test_scale_factor_correction(self):
        """Ověření škálovacího faktoru 1.05 (v_korig = v_raw / 1.05)."""
        self.tracker.update(speed_left=1050.0, speed_right=1050.0)
        self.assertAlmostEqual(self.tracker.raw_speed, 1050.0, places=1)
        self.assertAlmostEqual(self.tracker.speed, 1000.0, places=1)

    def test_zupt_stationary_detection(self):
        """Ověření detekce klidu při nulovém posunu kroků a rychlosti < 20 mm/s."""
        # 1. Změna kroků -> v pohybu
        self.tracker.update(speed_left=300.0, speed_right=300.0, left_steps=100, right_steps=100)
        self.assertFalse(self.tracker.is_stationary)

        # 2. První vzorek stání
        self.tracker.update(speed_left=0.0, speed_right=0.0, left_steps=100, right_steps=100)
        time.sleep(0.1)
        # 3. Druhý vzorek po > 80 ms se stejnými kroky
        self.tracker.update(speed_left=0.0, speed_right=0.0, left_steps=100, right_steps=100)
        self.assertTrue(self.tracker.is_stationary)

        # 4. Pohyb obnoven
        self.tracker.update(speed_left=500.0, speed_right=500.0, left_steps=150, right_steps=150)
        self.assertFalse(self.tracker.is_stationary)

    def test_fallback_without_steps(self):
        """Ověření chování, pokud kroky nejsou k dispozici (jen rychlost)."""
        self.tracker.update(speed_left=0.5, speed_right=0.5)
        self.assertTrue(self.tracker.is_stationary)

        self.tracker.update(speed_left=50.0, speed_right=50.0)
        self.assertFalse(self.tracker.is_stationary)

    def test_diagnostics(self):
        """Ověření struktury diagnostiky odometrie."""
        self.tracker.update(speed_left=525.0, speed_right=525.0, left_steps=10, right_steps=10)
        diag = self.tracker.get_diagnostics()
        self.assertAlmostEqual(diag["raw_speed"], 525.0)
        self.assertAlmostEqual(diag["corrected_speed"], 500.0)
        self.assertEqual(diag["scale_factor"], 1.05)
        self.assertIn("is_stationary", diag)
        self.assertEqual(diag["left_steps"], 10)
        self.assertEqual(diag["right_steps"], 10)


if __name__ == '__main__':
    unittest.main()
