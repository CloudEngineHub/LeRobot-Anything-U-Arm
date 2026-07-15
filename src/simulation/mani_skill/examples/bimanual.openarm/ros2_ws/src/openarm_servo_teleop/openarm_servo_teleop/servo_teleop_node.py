"""Publish dual 8-channel servo leader motion to OpenArm v1 controllers."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PROJECT_ROOT = Path("/mnt/d/openarm")
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from servo_zero import ArmReader, ArmState  # noqa: E402


ARM_ORDER = ("left", "right")
GRIPPER_MIN_M = 0.0
GRIPPER_MAX_M = 0.044
JOINT_RANGES = {
    "left": [
        (-3.490659, 1.396263),
        (-3.316125, 0.174533),
        (-1.570796, 1.570796),
        (0.0, 2.443461),
        (-1.570796, 1.570796),
        (-0.785398, 0.785398),
        (-1.570796, 1.570796),
    ],
    "right": [
        (-1.396263, 3.490659),
        (-0.174533, 3.316125),
        (-1.570796, 1.570796),
        (0.0, 2.443461),
        (-1.570796, 1.570796),
        (-0.785398, 0.785398),
        (-1.570796, 1.570796),
    ],
}


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def signed_angle_delta_deg(angle: float, reference: float) -> float:
    """Return the shortest signed angle delta, including across 0/360 degrees."""
    return (float(angle) - float(reference) + 180.0) % 360.0 - 180.0


def calibrated_trigger_ratio(
    angle: float,
    released_angle: float,
    pressed_span_deg: float,
    deadband_ratio: float,
) -> float:
    """Map a calibrated trigger angle to a clamped 0 (released) to 1 (pressed)."""
    if abs(pressed_span_deg) < 1e-9:
        raise ValueError("Trigger calibration span must be non-zero")
    ratio = clamp(
        signed_angle_delta_deg(angle, released_angle) / pressed_span_deg,
        0.0,
        1.0,
    )
    if ratio <= deadband_ratio:
        return 0.0
    return (ratio - deadband_ratio) / (1.0 - deadband_ratio)


def load_mapping(path: Path) -> dict[str, dict]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for arm in ARM_ORDER:
        records = raw[arm]["joints"]
        by_name = {record["joint_name"]: record for record in records}
        names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        if set(by_name) != set(names):
            raise ValueError(f"Invalid {arm} joint names in {path}")
        joints = []
        used_ids = []
        for name in names:
            record = by_name[name]
            servo_id = int(record["servo_id"])
            sign = int(record["sign"])
            if servo_id not in range(8) or sign not in (-1, 1):
                raise ValueError(f"Invalid servo_id/sign for {name}")
            joints.append(
                {
                    "name": name,
                    "servo_id": servo_id,
                    "sign": sign,
                    "scale": float(record["scale"]),
                    "home": math.radians(float(record["home_deg"])),
                    "max_delta": math.radians(float(record["max_delta_deg"])),
                }
            )
            used_ids.append(servo_id)
        gripper = raw[arm]["gripper"]
        gripper_id = int(gripper["servo_id"])
        gripper_sign = int(gripper["sign"])
        if sorted(used_ids + [gripper_id]) != list(range(8)):
            raise ValueError(f"{arm} must use servo IDs 0-7 exactly once")
        if gripper_sign not in (-1, 1):
            raise ValueError(f"Invalid gripper sign for {arm}")
        open_m = float(gripper.get("open_m", GRIPPER_MAX_M))
        closed_m = float(gripper.get("closed_m", GRIPPER_MIN_M))
        deadband_ratio = float(gripper.get("deadband_ratio", 0.05))
        minimum_travel_deg = float(gripper.get("minimum_travel_deg", 8.0))
        hold_seconds = float(gripper.get("hold_seconds", 0.75))
        stability_deg = float(gripper.get("stability_deg", 1.0))
        if not GRIPPER_MIN_M <= open_m <= GRIPPER_MAX_M:
            raise ValueError(f"Invalid open_m for {arm} gripper")
        if not GRIPPER_MIN_M <= closed_m <= GRIPPER_MAX_M or open_m == closed_m:
            raise ValueError(f"Invalid closed_m for {arm} gripper")
        if not 0.0 <= deadband_ratio < 1.0:
            raise ValueError(f"Invalid deadband_ratio for {arm} gripper")
        if not 0.0 < minimum_travel_deg < 180.0:
            raise ValueError(f"Invalid minimum_travel_deg for {arm} gripper")
        if hold_seconds <= 0.0 or stability_deg <= 0.0:
            raise ValueError(f"Invalid hold/stability setting for {arm} gripper")
        result[arm] = {
            "joints": joints,
            "gripper": {
                "servo_id": gripper_id,
                # Legacy fields remain in the shared JSON for MuJoCo v1 compatibility.
                "sign": gripper_sign,
                "m_per_deg": float(gripper["m_per_deg"]),
                "home": float(gripper["home_m"]),
                "max_delta": float(gripper["max_delta_m"]),
                "open": open_m,
                "closed": closed_m,
                "deadband_ratio": deadband_ratio,
                "minimum_travel_deg": minimum_travel_deg,
                "hold_seconds": hold_seconds,
                "stability_deg": stability_deg,
            },
        }
    return result


class ServoTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("openarm_servo_teleop")
        self.declare_parameter("left_port", "/dev/ttyUSB0")
        self.declare_parameter("right_port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("mapping_file", str(PROJECT_ROOT / "servo_teleop" / "joint_mapping_v1.json"))
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("calibration_seconds", 2.0)
        self.declare_parameter("trigger_calibration_timeout", 30.0)
        self.declare_parameter("signal_timeout", 0.5)
        self.declare_parameter("release_torque", True)

        self.mapping = load_mapping(Path(self.get_parameter("mapping_file").value))
        self.signal_timeout = float(self.get_parameter("signal_timeout").value)
        self.status_publisher = self.create_publisher(String, "/servo_teleop/status", 10)
        self.stop_event = threading.Event()
        self.states = {arm: ArmState() for arm in ARM_ORDER}
        baudrate = int(self.get_parameter("baudrate").value)
        release_torque = bool(self.get_parameter("release_torque").value)
        self.readers = [
            ArmReader("Left", str(self.get_parameter("left_port").value), baudrate, 0.2, 0.03, self.states["left"], self.stop_event, release_torque, 1.0),
            ArmReader("Right", str(self.get_parameter("right_port").value), baudrate, 0.2, 0.03, self.states["right"], self.stop_event, release_torque, 1.0),
        ]
        for reader in self.readers:
            reader.start()

        try:
            self.zero = self._collect_zero(
                float(self.get_parameter("calibration_seconds").value)
            )
            self.trigger_spans = self._collect_trigger_spans(
                float(self.get_parameter("trigger_calibration_timeout").value)
            )
        except Exception:
            self.stop_event.set()
            for reader in self.readers:
                reader.join(timeout=1.0)
            raise
        self.arm_publishers = {
            arm: self.create_publisher(JointTrajectory, f"/{arm}_joint_trajectory_controller/joint_trajectory", 10)
            for arm in ARM_ORDER
        }
        self.gripper_publishers = {
            arm: self.create_publisher(JointTrajectory, f"/{arm}_gripper_controller/joint_trajectory", 10)
            for arm in ARM_ORDER
        }
        self.target_publisher = self.create_publisher(JointState, "/servo_teleop/target_joint_states", 10)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_targets)
        self.last_timeout_warning = 0.0
        self.get_logger().info(
            "Calibration complete; publishing OpenArm v1 targets "
            "(校准完成，正在发布 OpenArm v1 目标)"
        )

    def _snapshot(self, arm: str):
        state = self.states[arm]
        with state.lock:
            angles = state.angles.copy()
            scans = state.scans
            updated = state.last_update
        if any(value is None for value in angles):
            return None, scans, updated
        return [float(value) for value in angles], scans, updated

    def _collect_zero(self, seconds: float) -> dict[str, list[float]]:
        samples = {arm: [] for arm in ARM_ORDER}
        last_scan = {arm: -1 for arm in ARM_ORDER}
        valid_since = None
        deadline = time.monotonic() + 12.0
        self.get_logger().info(
            "Keep both leader arms still and both triggers fully released during zero calibration"
        )
        last_status = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_status >= 0.25:
                self.status_publisher.publish(String(data="calibrating_released"))
                last_status = now
            failures = [reader for reader in self.readers if reader.startup_error is not None]
            if failures:
                raise RuntimeError("; ".join(str(reader.startup_error) for reader in failures))
            all_valid = True
            for arm in ARM_ORDER:
                angles, scan, updated = self._snapshot(arm)
                if angles is None or time.monotonic() - updated > 0.5:
                    all_valid = False
                    continue
                if scan != last_scan[arm]:
                    samples[arm].append(angles)
                    last_scan[arm] = scan
            if all_valid:
                valid_since = valid_since or time.monotonic()
                if time.monotonic() - valid_since >= seconds:
                    return {
                        arm: [statistics.median(channel) for channel in zip(*samples[arm])]
                        for arm in ARM_ORDER
                    }
            else:
                valid_since = None
            time.sleep(0.01)
        raise TimeoutError("Timed out waiting for both 8-channel leaders")

    def _collect_trigger_spans(self, timeout: float) -> dict[str, float]:
        self.get_logger().info(
            "Press both triggers fully and hold them still for calibration "
            "(请将左右扳机按到底并保持不动，进行校准)"
        )
        recent = {arm: deque() for arm in ARM_ORDER}
        last_scan = {arm: -1 for arm in ARM_ORDER}
        spans = {}
        deadline = time.monotonic() + timeout
        last_status = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_status >= 0.25:
                self.status_publisher.publish(String(data="calibrating_pressed"))
                last_status = now
            failures = [reader for reader in self.readers if reader.startup_error is not None]
            if failures:
                raise RuntimeError("; ".join(str(reader.startup_error) for reader in failures))

            for arm in ARM_ORDER:
                if arm in spans:
                    continue
                angles, scan, updated = self._snapshot(arm)
                if angles is None or now - updated > 0.5 or scan == last_scan[arm]:
                    continue
                last_scan[arm] = scan
                gripper = self.mapping[arm]["gripper"]
                servo_id = gripper["servo_id"]
                travel = signed_angle_delta_deg(
                    angles[servo_id], self.zero[arm][servo_id]
                )
                samples = recent[arm]
                if abs(travel) < gripper["minimum_travel_deg"]:
                    samples.clear()
                    continue

                samples.append((now, travel))
                hold_seconds = gripper["hold_seconds"]
                while samples and now - samples[0][0] > hold_seconds:
                    samples.popleft()
                if len(samples) < 3 or samples[-1][0] - samples[0][0] < hold_seconds * 0.9:
                    continue
                values = [sample[1] for sample in samples]
                if max(values) - min(values) > gripper["stability_deg"]:
                    continue

                spans[arm] = statistics.median(values)
                direction = "increases" if spans[arm] > 0.0 else "decreases"
                self.get_logger().info(
                    f"{arm.capitalize()} trigger calibrated: angle {direction} "
                    f"by {abs(spans[arm]):.2f} deg"
                )

            if len(spans) == len(ARM_ORDER):
                self.get_logger().info(
                    "Trigger calibration complete; release the triggers to open the grippers "
                    "(扳机校准完成，请松开扳机以打开夹爪)"
                )
                return spans
            time.sleep(0.01)

        missing = ", ".join(arm for arm in ARM_ORDER if arm not in spans)
        raise TimeoutError(
            f"Timed out calibrating fully pressed triggers: {missing or 'unknown'}"
        )

    def _targets(self, arm: str, angles: list[float]) -> tuple[list[float], float]:
        joint_targets = []
        for index, record in enumerate(self.mapping[arm]["joints"]):
            servo_id = record["servo_id"]
            delta = math.radians(
                signed_angle_delta_deg(angles[servo_id], self.zero[arm][servo_id])
            )
            delta = clamp(record["sign"] * record["scale"] * delta, -record["max_delta"], record["max_delta"])
            lower, upper = JOINT_RANGES[arm][index]
            joint_targets.append(clamp(record["home"] + delta, lower, upper))
        gripper = self.mapping[arm]["gripper"]
        servo_id = gripper["servo_id"]
        pressed_ratio = calibrated_trigger_ratio(
            angles[servo_id],
            self.zero[arm][servo_id],
            self.trigger_spans[arm],
            gripper["deadband_ratio"],
        )
        gripper_target = gripper["open"] + pressed_ratio * (
            gripper["closed"] - gripper["open"]
        )
        return joint_targets, clamp(gripper_target, GRIPPER_MIN_M, GRIPPER_MAX_M)

    def _trajectory(self, names: list[str], positions: list[float]) -> JointTrajectory:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=50_000_000)
        message.points = [point]
        return message

    def _publish_targets(self) -> None:
        snapshots = {}
        oldest_age = 0.0
        for arm in ARM_ORDER:
            angles, _, updated = self._snapshot(arm)
            age = time.monotonic() - updated if updated else math.inf
            oldest_age = max(oldest_age, age)
            if angles is None or age > self.signal_timeout:
                now = time.monotonic()
                if now - self.last_timeout_warning > 1.0:
                    self.get_logger().warning(f"Serial timeout; holding publishers (age={oldest_age:.3f}s)")
                    self.last_timeout_warning = now
                self.status_publisher.publish(String(data="serial_timeout"))
                return
            snapshots[arm] = angles

        target_state = JointState()
        target_state.header.stamp = self.get_clock().now().to_msg()
        for arm in ARM_ORDER:
            joints, gripper = self._targets(arm, snapshots[arm])
            joint_names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
            self.arm_publishers[arm].publish(self._trajectory(joint_names, joints))
            finger_name = f"openarm_{arm}_finger_joint1"
            self.gripper_publishers[arm].publish(self._trajectory([finger_name], [gripper]))
            target_state.name.extend(joint_names + [finger_name])
            target_state.position.extend(joints + [gripper])
        self.target_publisher.publish(target_state)
        self.status_publisher.publish(String(data="tracking"))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        for reader in self.readers:
            reader.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ServoTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
