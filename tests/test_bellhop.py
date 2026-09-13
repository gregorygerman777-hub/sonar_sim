import tempfile
import sys
import unittest

sys.path[:0] = [".", "python"]

import bellhop


class BellhopAdapterChecks(unittest.TestCase):
    def test_external_solver_produces_rays_and_physical_direct_delay(self):
        try:
            bellhop.executable_path()
        except FileNotFoundError:
            self.skipTest("external BELLHOP executable is not installed")
        environment = bellhop.BellhopEnvironment(
            frequency_hz=30000.0,
            sound_speed_surface_mps=1500.0,
            sound_speed_bottom_mps=1500.0,
            ray_count=81,
            receiver_count=11,
        )
        rays, arrivals, _ = bellhop.solve(environment, tempfile.mkdtemp(prefix="bellhop_test_"))
        self.assertEqual(len(rays["rays"]), 81)
        far = arrivals["records"][-1]["arrivals"]
        direct = [item for item in far
                  if item["top_bounces"] == 0 and item["bottom_bounces"] == 0]
        self.assertTrue(direct)
        expected = environment.max_range_m / 1500.0
        self.assertAlmostEqual(min(item["delay_s"] for item in direct), expected, places=7)

    def test_environment_file_contains_requested_physical_values(self):
        environment = bellhop.BellhopEnvironment(
            frequency_hz=42000.0, water_depth_m=44.0, source_depth_m=7.0,
            receiver_depth_m=9.0, max_range_m=230.0,
        )
        directory = tempfile.mkdtemp(prefix="bellhop_env_test_")
        path = bellhop.write_environment(f"{directory}/case.env", environment, "R")
        text = path.read_text()
        self.assertIn("42000", text)
        self.assertIn("51 0.0 44", text)
        self.assertIn("7 /", text)
        self.assertIn("9 /", text)
        self.assertIn("0.001 0.23 /", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
