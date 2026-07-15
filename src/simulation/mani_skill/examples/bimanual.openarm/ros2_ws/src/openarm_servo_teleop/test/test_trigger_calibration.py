import unittest
from pathlib import Path

from openarm_servo_teleop.servo_teleop_node import (
    calibrated_trigger_ratio,
    load_mapping,
    signed_angle_delta_deg,
)


class TriggerCalibrationTest(unittest.TestCase):
    def test_project_mapping_enables_calibrated_grippers(self):
        mapping = load_mapping(
            Path("/mnt/d/openarm/servo_teleop/joint_mapping_v1.json")
        )
        for arm in ("left", "right"):
            gripper = mapping[arm]["gripper"]
            self.assertEqual(gripper["servo_id"], 7)
            self.assertEqual(gripper["open"], 0.044)
            self.assertEqual(gripper["closed"], 0.0)
            self.assertGreaterEqual(gripper["minimum_travel_deg"], 1.0)

    def test_signed_delta_wraps_across_zero(self):
        self.assertAlmostEqual(signed_angle_delta_deg(2.0, 358.0), 4.0)
        self.assertAlmostEqual(signed_angle_delta_deg(358.0, 2.0), -4.0)

    def test_increasing_trigger_maps_to_zero_and_one(self):
        self.assertEqual(calibrated_trigger_ratio(100.0, 100.0, 25.0, 0.0), 0.0)
        self.assertAlmostEqual(
            calibrated_trigger_ratio(112.5, 100.0, 25.0, 0.0), 0.5
        )
        self.assertEqual(calibrated_trigger_ratio(125.0, 100.0, 25.0, 0.0), 1.0)

    def test_decreasing_trigger_uses_negative_calibration_span(self):
        self.assertEqual(calibrated_trigger_ratio(100.0, 100.0, -25.0, 0.0), 0.0)
        self.assertAlmostEqual(
            calibrated_trigger_ratio(87.5, 100.0, -25.0, 0.0), 0.5
        )
        self.assertEqual(calibrated_trigger_ratio(75.0, 100.0, -25.0, 0.0), 1.0)

    def test_deadband_is_removed_and_remaining_range_is_rescaled(self):
        self.assertEqual(calibrated_trigger_ratio(101.0, 100.0, 25.0, 0.05), 0.0)
        self.assertAlmostEqual(
            calibrated_trigger_ratio(112.5, 100.0, 25.0, 0.05),
            (0.5 - 0.05) / 0.95,
        )
        self.assertEqual(calibrated_trigger_ratio(125.0, 100.0, 25.0, 0.05), 1.0)

    def test_ratio_clamps_outside_calibrated_range(self):
        self.assertEqual(calibrated_trigger_ratio(90.0, 100.0, 25.0, 0.0), 0.0)
        self.assertEqual(calibrated_trigger_ratio(140.0, 100.0, 25.0, 0.0), 1.0)


if __name__ == "__main__":
    unittest.main()
