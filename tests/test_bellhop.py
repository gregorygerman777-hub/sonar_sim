import tempfile
import sys
import unittest

import numpy as np

sys.path[:0] = [".", "python"]

import bellhop


class BellhopAdapterChecks(unittest.TestCase):
    def test_two_way_path_pairing_has_object_ghost_and_mirror_ranges(self):
        arrivals = [
            {"amplitude": 1.0, "delay_s": 0.08,
             "top_bounces": 0, "bottom_bounces": 0},
            {"amplitude": 0.5, "delay_s": 0.10,
             "top_bounces": 1, "bottom_bounces": 0},
        ]
        paths = bellhop.two_way_paths(arrivals)
        self.assertEqual(len(paths), 4)
        by_range = {}
        for path in paths:
            key = round(path["apparent_range_m"], 6)
            by_range.setdefault(key, []).append(path)

        self.assertEqual(sorted(by_range), [120.0, 135.0, 150.0])
        self.assertAlmostEqual(by_range[120.0][0]["relative_intensity"], 1.0)
        self.assertEqual(len(by_range[135.0]), 2)
        self.assertTrue(all(path["mixed"] for path in by_range[135.0]))
        self.assertTrue(all(abs(path["relative_intensity"] - 0.25) < 1e-12
                            for path in by_range[135.0]))
        self.assertAlmostEqual(by_range[150.0][0]["relative_intensity"], 0.0625)

    def test_two_way_image_coupling_deposits_at_eigenray_ranges(self):
        arrivals = [
            {"amplitude": 1.0, "delay_s": 0.08,
             "top_bounces": 0, "bottom_bounces": 0},
            {"amplitude": 0.5, "delay_s": 0.10,
             "top_bounces": 1, "bottom_bounces": 0},
        ]
        ranges = np.arange(0.5, 160.0, 1.0)
        image = np.zeros((2, len(ranges)))
        source = int(np.argmin(np.abs(ranges - 120.0)))
        image[0, source] = 2.0
        result = {"receiver_ranges_m": np.array([120.0]),
                  "records": [{"arrivals": arrivals}]}

        coupled = bellhop.apply_two_way_multipath(
            image, ranges, result, source_depth_m=5.0, target_depth_m=5.0)
        at = lambda value: coupled[0, int(np.argmin(np.abs(ranges - value)))]
        self.assertAlmostEqual(at(120.0), 2.0)
        self.assertAlmostEqual(at(135.0), 1.0)
        self.assertAlmostEqual(at(150.0), 0.125)
        self.assertEqual(float(coupled[1].sum()), 0.0)

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
