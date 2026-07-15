"""Drive the OpenArm v1 MuJoCo model from two 8-DOF serial leaders."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import math
from pathlib import Path
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from mujoco_teleop import (
    ARM_ORDER,
    ArmReader,
    ArmState,
    collect_zero,
    configure_lighting,
    read_state,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
JOINT_COUNT = 7
KP = np.asarray([230.0, 230.0, 190.0, 190.0, 30.0, 30.0, 30.0])
KD = np.asarray([2.7, 2.7, 2.2, 2.2, 1.5, 1.5, 1.5])
FINGER_KP = 1000.0
FINGER_KD = 20.0
FINGER_FORCE_LIMIT = 333.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Teleoperate OpenArm v1 in MuJoCo from two serial leaders."
    )
    parser.add_argument("--left-port", default="/dev/ttyUSB0")
    parser.add_argument("--right-port", default="/dev/ttyUSB1")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--response-wait", type=float, default=0.03)
    parser.add_argument("--startup-delay", type=float, default=0.2)
    parser.add_argument(
        "--release-torque",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--torque-release-settle", type=float, default=1.0)
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--calibration-timeout", type=float, default=10.0)
    parser.add_argument("--signal-timeout", type=float, default=0.5)
    parser.add_argument("--filter-tau", type=float, default=0.08)
    parser.add_argument("--max-joint-speed", type=float, default=2.0, help="rad/s")
    parser.add_argument(
        "--max-gripper-speed", type=float, default=0.08, help="m/s"
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=PROJECT_ROOT / "openarm_mujoco" / "v1" / "scene.xml",
    )
    parser.add_argument(
        "--mapping",
        type=Path,
        default=Path(__file__).with_name("joint_mapping_v1.json"),
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def load_mapping(path: Path) -> dict[str, dict[str, np.ndarray | float | int]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    result: dict[str, dict[str, np.ndarray | float | int]] = {}

    for arm in ARM_ORDER:
        if arm not in raw:
            raise ValueError(f"Mapping is missing the '{arm}' section")
        joint_records = raw[arm].get("joints")
        if not isinstance(joint_records, list) or len(joint_records) != JOINT_COUNT:
            raise ValueError(f"{arm}.joints must contain exactly 7 entries")
        records_by_name = {record.get("joint_name"): record for record in joint_records}
        expected_names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        if set(records_by_name) != set(expected_names):
            raise ValueError(
                f"{arm}.joints must name each of: {', '.join(expected_names)}"
            )

        servo_ids = []
        signs = []
        scales = []
        homes = []
        max_deltas = []
        for name in expected_names:
            record = records_by_name[name]
            servo_id = int(record.get("servo_id"))
            sign = int(record.get("sign"))
            scale = float(record.get("scale"))
            home = float(record.get("home_deg"))
            max_delta = float(record.get("max_delta_deg"))
            if servo_id not in range(8):
                raise ValueError(f"{name}.servo_id must be between 0 and 7")
            if sign not in (-1, 1):
                raise ValueError(f"{name}.sign must be 1 or -1")
            if not all(math.isfinite(value) for value in (scale, home, max_delta)):
                raise ValueError(f"{name} contains a non-finite value")
            if scale <= 0 or max_delta <= 0:
                raise ValueError(f"{name} scale/max_delta_deg must be positive")
            servo_ids.append(servo_id)
            signs.append(sign)
            scales.append(scale)
            homes.append(home)
            max_deltas.append(max_delta)

        gripper = raw[arm].get("gripper")
        if not isinstance(gripper, dict):
            raise ValueError(f"{arm}.gripper must be an object")
        gripper_servo_id = int(gripper.get("servo_id"))
        gripper_sign = int(gripper.get("sign"))
        gripper_scale = float(gripper.get("m_per_deg"))
        gripper_home = float(gripper.get("home_m"))
        gripper_max_delta = float(gripper.get("max_delta_m"))
        if gripper_servo_id not in range(8):
            raise ValueError(f"{arm}.gripper.servo_id must be between 0 and 7")
        if gripper_sign not in (-1, 1):
            raise ValueError(f"{arm}.gripper.sign must be 1 or -1")
        if not all(
            math.isfinite(value)
            for value in (gripper_scale, gripper_home, gripper_max_delta)
        ):
            raise ValueError(f"{arm}.gripper contains a non-finite value")
        if gripper_scale <= 0 or gripper_max_delta <= 0:
            raise ValueError(f"{arm}.gripper scale/max_delta must be positive")
        if sorted(servo_ids + [gripper_servo_id]) != list(range(8)):
            raise ValueError(f"{arm} must use each servo_id from 0 through 7 once")

        values: dict[str, np.ndarray | float | int] = {
            "joint_servo_id": np.asarray(servo_ids, dtype=np.intp),
            "joint_sign": np.asarray(signs, dtype=float),
            "joint_scale": np.asarray(scales, dtype=float),
            "joint_home": np.radians(np.asarray(homes, dtype=float)),
            "joint_max_delta": np.radians(np.asarray(max_deltas, dtype=float)),
            "gripper_servo_id": gripper_servo_id,
            "gripper_sign": gripper_sign,
            "gripper_m_per_deg": gripper_scale,
            "gripper_home_m": gripper_home,
            "gripper_max_delta_m": gripper_max_delta,
        }
        result[arm] = values
    return result


def print_mapping(mapping: dict[str, dict[str, np.ndarray | float | int]]) -> None:
    for arm in ARM_ORDER:
        settings = mapping[arm]
        parts = [
            f"J{i + 1}<-servo{int(servo_id)}({int(sign):+d})"
            for i, (servo_id, sign) in enumerate(
                zip(settings["joint_servo_id"], settings["joint_sign"], strict=True)
            )
        ]
        parts.append(
            f"gripper<-servo{int(settings['gripper_servo_id'])}"
            f"({int(settings['gripper_sign']):+d})"
        )
        print(f"{arm.capitalize()} mapping: " + ", ".join(parts))


class V1ModelMap:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.joint_qpos: dict[str, np.ndarray] = {}
        self.joint_dof: dict[str, np.ndarray] = {}
        self.joint_actuator: dict[str, np.ndarray] = {}
        self.joint_range: dict[str, np.ndarray] = {}
        self.force_range: dict[str, np.ndarray] = {}
        self.finger_qpos: dict[str, np.ndarray] = {}
        self.finger_dof: dict[str, np.ndarray] = {}
        self.finger_actuator: dict[str, np.ndarray] = {}

        for arm in ARM_ORDER:
            joint_ids = np.asarray(
                [
                    self._id(
                        model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        f"openarm_{arm}_joint{i}",
                    )
                    for i in range(1, 8)
                ],
                dtype=np.intp,
            )
            actuator_ids = np.asarray(
                [
                    self._id(
                        model,
                        mujoco.mjtObj.mjOBJ_ACTUATOR,
                        f"{arm}_joint{i}_ctrl",
                    )
                    for i in range(1, 8)
                ],
                dtype=np.intp,
            )
            finger_joint_ids = np.asarray(
                [
                    self._id(
                        model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        f"openarm_{arm}_finger_joint{i}",
                    )
                    for i in (1, 2)
                ],
                dtype=np.intp,
            )
            finger_actuator_ids = np.asarray(
                [
                    self._id(
                        model,
                        mujoco.mjtObj.mjOBJ_ACTUATOR,
                        f"{arm}_finger{i}_ctrl",
                    )
                    for i in (1, 2)
                ],
                dtype=np.intp,
            )

            self.joint_qpos[arm] = model.jnt_qposadr[joint_ids].astype(np.intp)
            self.joint_dof[arm] = model.jnt_dofadr[joint_ids].astype(np.intp)
            self.joint_actuator[arm] = actuator_ids
            self.joint_range[arm] = model.jnt_range[joint_ids].copy()
            self.force_range[arm] = model.actuator_forcerange[actuator_ids].copy()
            self.finger_qpos[arm] = model.jnt_qposadr[finger_joint_ids].astype(
                np.intp
            )
            self.finger_dof[arm] = model.jnt_dofadr[finger_joint_ids].astype(
                np.intp
            )
            self.finger_actuator[arm] = finger_actuator_ids

        # The v1 XML compiles the left and right finger shortcuts differently.
        # Disable them and apply symmetric joint-space PD force below.
        for ids in self.finger_actuator.values():
            model.actuator_gainprm[ids, :] = 0.0
            model.actuator_biasprm[ids, :] = 0.0

    @staticmethod
    def _id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
        value = mujoco.mj_name2id(model, kind, name)
        if value < 0:
            raise ValueError(f"v1 model is missing '{name}'")
        return value


def initialize_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    model_map: V1ModelMap,
    mapping: dict[str, dict[str, np.ndarray | float | int]],
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    ranges: dict[str, np.ndarray] = {}
    homes = []
    mujoco.mj_resetData(model, data)
    for arm in ARM_ORDER:
        settings = mapping[arm]
        joint_home = np.clip(
            settings["joint_home"],
            model_map.joint_range[arm][:, 0],
            model_map.joint_range[arm][:, 1],
        )
        gripper_home = float(np.clip(settings["gripper_home_m"], 0.0, 0.044))
        data.qpos[model_map.joint_qpos[arm]] = joint_home
        data.qpos[model_map.finger_qpos[arm]] = gripper_home
        data.ctrl[model_map.joint_actuator[arm]] = 0.0
        data.ctrl[model_map.finger_actuator[arm]] = 0.0
        ranges[arm] = np.vstack(
            [model_map.joint_range[arm], np.asarray([0.0, 0.044])]
        )
        homes.append(np.concatenate([joint_home, [gripper_home]]))
    mujoco.mj_forward(model, data)
    return ranges, np.concatenate(homes)


def map_targets(
    states: dict[str, ArmState],
    zero: dict[str, np.ndarray],
    mapping: dict[str, dict[str, np.ndarray | float | int]],
    ranges: dict[str, np.ndarray],
    signal_timeout: float,
) -> tuple[dict[str, np.ndarray] | None, float]:
    now = time.monotonic()
    targets: dict[str, np.ndarray] = {}
    oldest_age = 0.0
    for arm in ARM_ORDER:
        angles, _, last_update = read_state(states[arm])
        age = now - last_update if last_update else math.inf
        oldest_age = max(oldest_age, age)
        if angles is None or age > signal_timeout:
            return None, oldest_age

        settings = mapping[arm]
        joint_servo_ids = settings["joint_servo_id"]
        joint_delta = np.radians(
            angles[joint_servo_ids] - zero[arm][joint_servo_ids]
        )
        joint_delta *= settings["joint_sign"] * settings["joint_scale"]
        joint_delta = np.clip(
            joint_delta,
            -settings["joint_max_delta"],
            settings["joint_max_delta"],
        )
        joint_target = settings["joint_home"] + joint_delta

        gripper_delta = (
            settings["gripper_sign"]
            * settings["gripper_m_per_deg"]
            * (
                angles[int(settings["gripper_servo_id"])]
                - zero[arm][int(settings["gripper_servo_id"])]
            )
        )
        gripper_delta = float(
            np.clip(
                gripper_delta,
                -settings["gripper_max_delta_m"],
                settings["gripper_max_delta_m"],
            )
        )
        gripper_target = settings["gripper_home_m"] + gripper_delta
        target = np.concatenate([joint_target, [gripper_target]])
        targets[arm] = np.clip(target, ranges[arm][:, 0], ranges[arm][:, 1])
    return targets, oldest_age


def apply_control(
    data: mujoco.MjData,
    model_map: V1ModelMap,
    filtered: np.ndarray,
) -> None:
    for arm, target in zip(
        ARM_ORDER, np.split(filtered, len(ARM_ORDER)), strict=True
    ):
        qpos = data.qpos[model_map.joint_qpos[arm]]
        qvel = data.qvel[model_map.joint_dof[arm]]
        torque = KP * (target[:7] - qpos) - KD * qvel
        torque = np.clip(
            torque,
            model_map.force_range[arm][:, 0],
            model_map.force_range[arm][:, 1],
        )
        data.ctrl[model_map.joint_actuator[arm]] = torque
        finger_qpos = data.qpos[model_map.finger_qpos[arm]]
        finger_qvel = data.qvel[model_map.finger_dof[arm]]
        finger_force = (
            FINGER_KP * (target[7] - finger_qpos) - FINGER_KD * finger_qvel
        )
        data.qfrc_applied[model_map.finger_dof[arm]] = np.clip(
            finger_force, -FINGER_FORCE_LIMIT, FINGER_FORCE_LIMIT
        )


def main() -> int:
    args = parse_args()
    if args.left_port == args.right_port:
        raise SystemExit("Left and right serial ports must be different")
    if min(
        args.response_wait,
        args.calibration_seconds,
        args.calibration_timeout,
        args.signal_timeout,
        args.max_joint_speed,
        args.max_gripper_speed,
    ) <= 0:
        raise SystemExit("Timing and speed values must be positive")

    mapping = load_mapping(args.mapping.resolve())
    print_mapping(mapping)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    configure_lighting(model)
    model_map = V1ModelMap(model)
    data = mujoco.MjData(model)
    ranges, filtered = initialize_pose(model, data, model_map, mapping)

    stop_event = threading.Event()
    states = {arm: ArmState() for arm in ARM_ORDER}
    readers = [
        ArmReader(
            "Left",
            args.left_port,
            args.baudrate,
            args.startup_delay,
            args.response_wait,
            states["left"],
            stop_event,
            args.release_torque,
            args.torque_release_settle,
        ),
        ArmReader(
            "Right",
            args.right_port,
            args.baudrate,
            args.startup_delay,
            args.response_wait,
            states["right"],
            stop_event,
            args.release_torque,
            args.torque_release_settle,
        ),
    ]
    for reader in readers:
        reader.start()

    try:
        zero = collect_zero(
            states,
            readers,
            stop_event,
            args.calibration_seconds,
            args.calibration_timeout,
        )
        print("V1 calibration complete. Move the leader arms to drive MuJoCo.")

        viewer_context = (
            nullcontext(None)
            if args.headless
            else mujoco.viewer.launch_passive(model, data)
        )
        started = time.monotonic()
        last_loop = started
        last_status = started
        signal_ok = True
        speed_limits = np.asarray(
            [args.max_joint_speed] * 7 + [args.max_gripper_speed]
            + [args.max_joint_speed] * 7
            + [args.max_gripper_speed]
        )

        with viewer_context as viewer:
            while viewer is None or viewer.is_running():
                loop_started = time.monotonic()
                dt = max(loop_started - last_loop, model.opt.timestep)
                last_loop = loop_started

                targets, signal_age = map_targets(
                    states, zero, mapping, ranges, args.signal_timeout
                )
                if targets is not None:
                    desired = np.concatenate([targets[arm] for arm in ARM_ORDER])
                    alpha = (
                        1.0
                        if args.filter_tau == 0
                        else 1.0 - math.exp(-dt / args.filter_tau)
                    )
                    filtered_target = filtered + alpha * (desired - filtered)
                    max_step = speed_limits * dt
                    filtered += np.clip(
                        filtered_target - filtered, -max_step, max_step
                    )
                    signal_ok = True
                else:
                    signal_ok = False

                apply_control(data, model_map, filtered)
                mujoco.mj_step(model, data)
                if viewer is not None:
                    viewer.sync()

                if loop_started - last_status >= 1.0:
                    state_text = "tracking" if signal_ok else "holding (serial timeout)"
                    print(f"V1 simulation: {state_text}, serial age={signal_age:.3f}s")
                    last_status = loop_started
                if args.duration is not None and loop_started - started >= args.duration:
                    break
                sleep_time = model.opt.timestep - (time.monotonic() - loop_started)
                if sleep_time > 0:
                    time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for reader in readers:
            reader.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
