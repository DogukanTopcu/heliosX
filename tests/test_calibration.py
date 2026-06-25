import math
import os
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import v2.new2 as turret


def make_linear_samples():
    samples = []
    for tilt in (1.0, 3.0, 5.0, 7.0, 9.0):
        for pan in (-4.5, -2.7, -0.9, 0.9, 2.7, 4.5):
            samples.append({
                "pan": pan,
                "tilt": tilt,
                "px": 320.0 + pan * 10.0,
                "py": 120.0 + tilt * 7.0,
                "jitter_px": 1.0,
                "weight": 1.0,
            })
    return samples


class CalibrationTests(unittest.TestCase):
    def tearDown(self):
        turret.HAS_PIXEL_MAP = False
        turret.CALIBRATION_SAMPLES = []
        turret.REACHABLE_HULL = None

    def test_grid_is_inset_from_mechanical_boundaries(self):
        calibrator = turret.AutoCalibrator()
        pans = sorted({round(p, 1) for p, _ in calibrator.grid})
        tilts = sorted({round(t, 1) for _, t in calibrator.grid})
        self.assertEqual(pans, [-4.5, -2.7, -0.9, 0.9, 2.7, 4.5])
        self.assertEqual(tilts, [1.0, 3.0, 5.0, 7.0, 9.0])

    def test_dynamic_span_threshold_matches_current_ranges(self):
        min_x, min_y = turret._minimum_calibration_span()
        self.assertAlmostEqual(min_x, 64.97, places=1)
        self.assertAlmostEqual(min_y, 49.22, places=1)

    def test_jitter_is_weighted_then_rejected(self):
        stable = [(10.0, 10.0, 4.0)] * 10
        summary, error = turret._summarize_dot_samples(stable)
        self.assertIsNone(error)
        self.assertEqual(summary["weight"], 1.0)

        noisy = [(10.0 + math.cos(i) * 8.0, 10.0 + math.sin(i) * 8.0, 4.0)
                 for i in range(10)]
        summary, error = turret._summarize_dot_samples(noisy)
        self.assertIsNone(error)
        self.assertGreaterEqual(summary["weight"], 0.25)
        self.assertLess(summary["weight"], 1.0)

        unstable = [(10.0 + math.cos(i) * 12.0, 10.0 + math.sin(i) * 12.0, 4.0)
                    for i in range(10)]
        summary, error = turret._summarize_dot_samples(unstable)
        self.assertIsNone(summary)
        self.assertIn("jitter", error)

    def test_boundary_home_summary_is_more_tolerant_than_regular_summary(self):
        home_noisy = [(10.0 + math.cos(i) * 11.0, 10.0 + math.sin(i) * 11.0, 4.0)
                      for i in range(10)]
        summary, error = turret._summarize_dot_samples(home_noisy)
        self.assertIsNone(summary)
        self.assertIn("jitter", error)

        summary, error = turret._summarize_home_dot_samples(home_noisy)
        self.assertIsNone(error)
        self.assertIsNotNone(summary)
        self.assertEqual(summary["weight"], 0.25)

    def test_linear_model_is_preferred_for_linear_data(self):
        result = turret._fit_pixel_to_servo(make_linear_samples())
        self.assertEqual(result["model_type"], "linear")
        self.assertLess(result["reprojection_error_px"], 0.001)
        self.assertLess(result["rms_pan_deg"], 0.001)
        self.assertLess(result["rms_tilt_deg"], 0.001)
        usable, reason = turret._is_pixel_map_usable(result)
        self.assertTrue(usable, reason)

    def test_quadratic_is_selected_only_for_material_nonlinearity(self):
        samples = []
        for tilt in (1.0, 3.0, 5.0, 7.0, 9.0):
            for pan in (-4.5, -2.7, -0.9, 0.9, 2.7, 4.5):
                samples.append({
                    "pan": pan,
                    "tilt": tilt,
                    "px": 320.0 + 12.0 * pan + 1.5 * pan * pan + 0.4 * pan * tilt,
                    "py": 100.0 + 8.0 * tilt + 0.5 * pan * pan,
                    "jitter_px": 1.0,
                    "weight": 1.0,
                })
        result = turret._fit_pixel_to_servo(samples)
        self.assertEqual(result["model_type"], "quadratic")
        improvement = (result["linear_reprojection_error_px"]
                       - result["quadratic_reprojection_error_px"])
        self.assertGreaterEqual(improvement, turret.AUTOCAL_QUADRATIC_MIN_IMPROVEMENT_PX)

    def test_reachable_hull_blocks_extrapolation_and_idw_maps_inside(self):
        result = turret._fit_pixel_to_servo(make_linear_samples())
        turret.HAS_PIXEL_MAP = True
        turret.CALIBRATION_SAMPLES = result["samples"]
        turret.REACHABLE_HULL = np.asarray(result["reachable_hull"], dtype=np.float32)

        self.assertTrue(turret.is_reachable_pixel(320.0, 155.0))
        self.assertFalse(turret.is_reachable_pixel(20.0, 20.0))
        pan, tilt = turret.pixel_to_servo(320.0, 155.0)
        self.assertAlmostEqual(pan, 0.0, delta=0.25)
        self.assertAlmostEqual(tilt, 5.0, delta=0.25)

    def test_backlash_move_uses_lower_angle_preapproach(self):
        turret.current_pan_deg = 0.0
        turret.current_tilt_deg = 5.0
        with patch.object(turret, "smooth_move") as move:
            turret.backlash_compensated_move(3.0, 7.0)
        first, final = move.call_args_list
        self.assertAlmostEqual(first.args[0], 2.6)
        self.assertAlmostEqual(first.args[1], 6.6)
        self.assertEqual(final.args, (3.0, 7.0))

    def test_camera_or_range_change_invalidates_map(self):
        data = {
            "schema_version": turret.CALIBRATION_SCHEMA_VERSION,
            "capture_width": turret.CAPTURE_W,
            "capture_height": turret.CAPTURE_H,
            "process_width": turret.PROCESS_W,
            "process_height": turret.PROCESS_H,
            "camera_exposure_us": turret.EXPOSURE_TIME_US,
            "camera_gain": turret.ANALOGUE_GAIN,
            "pan_range": [turret.PAN_MIN_DEG, turret.PAN_MAX_DEG],
            "tilt_range": [turret.TILT_MIN_DEG, turret.TILT_MAX_DEG],
        }
        self.assertIsNone(turret._calibration_compatibility_error(data))
        data["process_width"] += 1
        self.assertIn("process_width", turret._calibration_compatibility_error(data))

    def test_boundary_safe_area_is_shrunk_from_discovered_limits(self):
        safe = turret._boundary_safe_area({
            "left": {"pan": -4.5, "tilt": 5.0, "px": 100.0, "py": 170.0},
            "right": {"pan": 4.5, "tilt": 5.0, "px": 540.0, "py": 170.0},
            "up": {"pan": 0.0, "tilt": 9.0, "px": 320.0, "py": 70.0},
            "down": {"pan": 0.0, "tilt": 1.0, "px": 320.0, "py": 280.0},
        })
        self.assertIsNotNone(safe)
        self.assertAlmostEqual(safe["pan_min"], -4.25)
        self.assertAlmostEqual(safe["pan_max"], 4.25)
        self.assertAlmostEqual(safe["tilt_min"], 1.25)
        self.assertAlmostEqual(safe["tilt_max"], 8.75)
        self.assertEqual(len(safe["safe_polygon"]), 4)
        self.assertGreaterEqual(len(safe["boundary_hull"]), 3)

    def test_boundary_grid_plan_stays_inside_safe_area(self):
        safe = turret._boundary_safe_area({
            "left": {"pan": -4.5, "tilt": 5.0, "px": 100.0, "py": 170.0},
            "right": {"pan": 4.5, "tilt": 5.0, "px": 540.0, "py": 170.0},
            "up": {"pan": 0.0, "tilt": 9.0, "px": 320.0, "py": 70.0},
            "down": {"pan": 0.0, "tilt": 1.0, "px": 320.0, "py": 280.0},
        })
        plan = turret._boundary_grid_plan(safe)
        self.assertEqual(len(plan), turret.AUTOCAL_GRID_PAN * turret.AUTOCAL_GRID_TILT)
        for point in plan:
            self.assertGreaterEqual(point["pan"], safe["pan_min"])
            self.assertLessEqual(point["pan"], safe["pan_max"])
            self.assertGreaterEqual(point["tilt"], safe["tilt_min"])
            self.assertLessEqual(point["tilt"], safe["tilt_max"])
            self.assertGreaterEqual(point["planned_px"], safe["x_min"])
            self.assertLessEqual(point["planned_px"], safe["x_max"])
            self.assertGreaterEqual(point["planned_py"], safe["y_min"])
            self.assertLessEqual(point["planned_py"], safe["y_max"])

    def test_saved_map_round_trips_with_schema_and_reachable_hull(self):
        result = turret._fit_pixel_to_servo(make_linear_samples())
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "calibration.json")
            with patch.object(turret, "CALIB_PATH", path):
                turret._save_pixel_map(result)
                turret.HAS_PIXEL_MAP = False
                turret.CALIBRATION_SAMPLES = []
                turret.REACHABLE_HULL = None
                turret._load_calibration()
        self.assertTrue(turret.HAS_PIXEL_MAP)
        self.assertEqual(turret.MAP_MODEL_TYPE, "linear")
        self.assertEqual(len(turret.CALIBRATION_SAMPLES), 30)
        self.assertGreaterEqual(len(turret.REACHABLE_HULL), 3)

    def test_boundary_grid_save_persists_mode_and_metadata(self):
        result = turret._fit_pixel_to_servo(make_linear_samples())
        result.update({
            "boundary_points": {
                "left": {"pan": -4.5, "tilt": 5.0, "px": 100.0, "py": 170.0},
                "right": {"pan": 4.5, "tilt": 5.0, "px": 540.0, "py": 170.0},
                "up": {"pan": 0.0, "tilt": 9.0, "px": 320.0, "py": 70.0},
                "down": {"pan": 0.0, "tilt": 1.0, "px": 320.0, "py": 280.0},
            },
            "safe_polygon": [[110.0, 80.0], [530.0, 80.0], [530.0, 270.0], [110.0, 270.0]],
            "generated_grid_points": [{"pan": 0.0, "tilt": 5.0, "planned_px": 320.0, "planned_py": 175.0}],
            "accepted_samples": result["samples"],
            "rejected_samples": [{"pan": 0.0, "tilt": 0.0, "reason": "frame kenarı"}],
            "home_sample": {"pan": 0.0, "tilt": 0.0, "px": 320.0, "py": 180.0},
        })
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "calibration.json")
            with patch.object(turret, "CALIB_PATH", path):
                turret._save_pixel_map(result, mode="boundary_grid")
                with open(path) as fh:
                    data = turret._json.load(fh)
                turret.HAS_PIXEL_MAP = False
                turret.CALIBRATION_SAMPLES = []
                turret.REACHABLE_HULL = None
                turret._load_calibration()
        self.assertEqual(data["mode"], "boundary_grid")
        self.assertIn("boundary_points", data)
        self.assertIn("generated_grid_points", data)
        self.assertTrue(turret.HAS_PIXEL_MAP)

    def test_boundary_grid_calibrator_discovers_area_and_fits_map(self):
        dummy = np.zeros((turret.PROCESS_H, turret.PROCESS_W, 3), dtype=np.uint8)
        with patch.object(turret, "AUTOCAL_REF_FR", 1), \
             patch.object(turret, "AUTOCAL_SETTLE_FR", 1), \
             patch.object(turret, "AUTOCAL_MEASURE_FR", 2), \
             patch.object(turret, "AUTOCAL_MIN_OBSERVATIONS", 1):
            calibrator = turret.BoundaryGridCalibrator()

            def fake_detect(_frame, _ref=None):
                if "measure" not in calibrator.phase:
                    return None
                pan, tilt = calibrator.current_target
                px = 320.0 + pan * 10.0
                py = 220.0 - tilt * 7.0
                return (px, py, 4.0)

            with patch.object(turret, "_detect_laser_dot", side_effect=fake_detect):
                cmd = None
                for _ in range(500):
                    cmd = calibrator.update(dummy)
                    if cmd["done"]:
                        break

        self.assertIsNotNone(cmd)
        self.assertTrue(cmd["done"])
        self.assertIsNotNone(cmd["result"])
        self.assertEqual(cmd["result"]["model_type"], "linear")
        self.assertEqual(set(cmd["result"]["boundary_points"]),
                         {"left", "right", "up", "down"})
        self.assertEqual(len(cmd["result"]["generated_grid_points"]),
                         turret.AUTOCAL_GRID_PAN * turret.AUTOCAL_GRID_TILT)

    def test_boundary_grid_calibrator_fails_when_home_dot_missing(self):
        dummy = np.zeros((turret.PROCESS_H, turret.PROCESS_W, 3), dtype=np.uint8)
        with patch.object(turret, "AUTOCAL_REF_FR", 1), \
             patch.object(turret, "AUTOCAL_SETTLE_FR", 1), \
             patch.object(turret, "AUTOCAL_MEASURE_FR", 2), \
             patch.object(turret, "AUTOCAL_MIN_OBSERVATIONS", 1), \
             patch.object(turret, "_detect_laser_dot", return_value=None):
            calibrator = turret.BoundaryGridCalibrator()
            cmd = None
            for _ in range(20):
                cmd = calibrator.update(dummy)
                if cmd["done"]:
                    break

        self.assertIsNotNone(cmd)
        self.assertTrue(cmd["done"])
        self.assertIsNone(cmd["result"])
        self.assertIn("home", cmd["status"].lower())


if __name__ == "__main__":
    unittest.main()
