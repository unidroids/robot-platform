# fusion/core_modules/test_geo_math.py
import math
import unittest

try:
    from .geo_math import norm_deg, angle_diff, circular_mean, geo_dist, compute_tangent_offset
except (ImportError, ValueError):
    from core_modules.geo_math import norm_deg, angle_diff, circular_mean, geo_dist, compute_tangent_offset


class TestGeoMath(unittest.TestCase):

    def test_norm_deg(self):
        self.assertAlmostEqual(norm_deg(0.0), 0.0)
        self.assertAlmostEqual(norm_deg(360.0), 0.0)
        self.assertAlmostEqual(norm_deg(90.0), 90.0)
        self.assertAlmostEqual(norm_deg(-10.0), 350.0)
        self.assertAlmostEqual(norm_deg(-370.0), 350.0)
        self.assertAlmostEqual(norm_deg(725.0), 5.0)

    def test_angle_diff(self):
        self.assertAlmostEqual(angle_diff(10.0, 0.0), 10.0)
        self.assertAlmostEqual(angle_diff(0.0, 10.0), -10.0)
        # Přechod přes 0°
        self.assertAlmostEqual(angle_diff(10.0, 350.0), 20.0)
        self.assertAlmostEqual(angle_diff(350.0, 10.0), -20.0)
        self.assertAlmostEqual(abs(angle_diff(180.0, 0.0)), 180.0)
        self.assertAlmostEqual(abs(angle_diff(181.0, 0.0)), 179.0)

    def test_circular_mean(self):
        self.assertAlmostEqual(circular_mean([]), 0.0)
        self.assertAlmostEqual(circular_mean([10.0, 20.0]), 15.0)
        # Přechod přes 0°: 350° a 10° má průměr 0° (resp. 360°)
        mean_cross = circular_mean([350.0, 10.0])
        self.assertTrue(math.isclose(mean_cross, 0.0, abs_tol=1e-5) or math.isclose(mean_cross, 360.0, abs_tol=1e-5))

    def test_geo_dist(self):
        self.assertAlmostEqual(geo_dist(0.0, 0.0, 0.0, 0.0), 0.0)
        # 1 stupeň zeměpisné šířky ~ 111139 m
        d = geo_dist(50.0, 14.0, 51.0, 14.0)
        self.assertAlmostEqual(d, 111139.0, delta=10.0)

    def test_compute_tangent_offset(self):
        # Nízká rychlost pod prahem 0.2 m/s -> nulová kompenzace
        self.assertEqual(compute_tangent_offset(0.32, 0.0, 0.1, 10.0), 0.0)

        # v = 0.5 m/s, wz = 6.0 deg/s:
        # GPS (x=0.32, y=0.0): beta = atan2(0.1047*0.32, 0.5) ~ 3.83°
        beta_gps = compute_tangent_offset(0.32, 0.0, 0.5, 6.0)
        self.assertAlmostEqual(beta_gps, 3.83, delta=0.05)

        # Master (x=0.25, y=0.24): beta = atan2(0.1047*0.25, 0.5 + 0.1047*0.24) ~ 2.85°
        beta_mst = compute_tangent_offset(0.25, 0.24, 0.5, 6.0)
        self.assertAlmostEqual(beta_mst, 2.85, delta=0.05)

        # Slave (x=0.25, y=-0.24): beta = atan2(0.1047*0.25, 0.5 - 0.1047*0.24) ~ 3.16°
        beta_slv = compute_tangent_offset(0.25, -0.24, 0.5, 6.0)
        self.assertAlmostEqual(beta_slv, 3.16, delta=0.05)


if __name__ == '__main__':
    unittest.main()
