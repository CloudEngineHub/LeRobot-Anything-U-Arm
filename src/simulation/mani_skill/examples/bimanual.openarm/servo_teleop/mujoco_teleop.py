"""Drive the OpenArm v2 MuJoCo model from two 8-DOF serial leaders."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import math
from pathlib import Path
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from servo_zero import ArmReader, ArmState  # noqa: E402


CHANNELS_PER_ARM = 8
ARM_ORDER = ("left", "right")
ACTUATOR_NAMES = {
    "left": [f"left_joint{i}_ctrl" for i in range(1, 8)]
    + ["left_finger1_ctrl"],
    "right": [f"right_joint{i}_ctrl" for i in range(1, 8)]
    + ["right_finger1_ctrl"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Teleoperate OpenArm v2 in MuJoCo from two serial leaders."
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
        help="send PULK to all 16 leader servos before calibration",
    )
    parser.add_argument("--torque-release-settle", type=float, default=1.0)
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--calibration-timeout", type=float, default=10.0)
    parser.add_argument("--signal-timeout", type=float, default=0.5)
    parser.add_argument("--filter-tau", type=float, default=0.08)
    parser.add_argument("--max-speed", type=float, default=2.0, help="rad/s")
    parser.add_argument(
        "--model",
        type=Path,
        default=PROJECT_ROOT / "openarm_mujoco" / "v2" / "pedestal.xml",
    )
    parser.add_argument(
        "--mapping",
        type=Path,
        default=Path(__file__).with_name("joint_mapping.json"),
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def load_mapping(path: Path) -> dict[str, dict[str, np.ndarray]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    mapping: dict[str, dict[str, np.ndarray]] = {}
    required = ("sign", "scale", "home_deg", "max_delta_deg")

    for arm in ARM_ORDER:
        if arm not in raw:
            raise ValueError(f"Mapping is missing the '{arm}' section")
        values: dict[str, np.ndarray] = {}
        for key in required:
            array = np.asarray(raw[arm].get(key), dtype=float)
            if array.shape != (CHANNELS_PER_ARM,):
                raise ValueError(f"{arm}.{key} must contain exactly 8 values")
            if not np.all(np.isfinite(array)):
                raise ValueError(f"{arm}.{key} contains a non-finite value")
            values[key] = array
        if np.any(values["sign"] == 0):
            raise ValueError(f"{arm}.sign cannot contain zero")
        if np.any(values["scale"] <= 0) or np.any(values["max_delta_deg"] <= 0):
            raise ValueError(f"{arm} scale and max_delta_deg values must be positive")
        values["home"] = np.radians(values.pop("home_deg"))
        values["max_delta"] = np.radians(values.pop("max_delta_deg"))
        mapping[arm] = values
    return mapping


def actuator_indices(model: mujoco.MjModel) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for arm in ARM_ORDER:
        indices = []
        for name in ACTUATOR_NAMES[arm]:
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if index < 0:
                raise ValueError(f"Model is missing actuator '{name}'")
            indices.append(index)
        result[arm] = np.asarray(indices, dtype=np.intp)
    return result


def configure_lighting(model: mujoco.MjModel) -> None:
    """Apply a bright, neutral setup that remains readable under WSLg."""
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
    model.vis.headlight.diffuse[:] = (0.85, 0.85, 0.85)
    model.vis.headlight.specular[:] = (0.15, 0.15, 0.15)
    if model.nlight:
        model.light_ambient[:] = np.maximum(model.light_ambient, 0.2)
        model.light_diffuse[:] = np.maximum(model.light_diffuse, 0.9)


def read_state(state: ArmState) -> tuple[np.ndarray | None, int, float]:
    with state.lock:
        angles = state.angles.copy()
        scans = state.scans
        last_update = state.last_update
    if any(angle is None for angle in angles):
        return None, scans, last_update
    return np.asarray(angles, dtype=float), scans, last_update


def collect_zero(
    states: dict[str, ArmState],
    readers: list[ArmReader],
    stop_event: threading.Event,
    sample_seconds: float,
    timeout: float,
) -> dict[str, np.ndarray]:
    samples: dict[str, list[np.ndarray]] = {arm: [] for arm in ARM_ORDER}
    last_scan = {arm: -1 for arm in ARM_ORDER}
    started = time.monotonic()
    valid_since: float | None = None

    print("Keep both leader arms still while their zero pose is sampled...")
    while time.monotonic() - started < timeout:
        failures = [reader for reader in readers if reader.startup_error is not None]
        if failures:
            details = "; ".join(
                f"{reader.arm_name}: {reader.startup_error}" for reader in failures
            )
            raise RuntimeError(f"Serial reader failed: {details}")
        if stop_event.is_set():
            raise RuntimeError("Serial readers stopped before calibration")

        all_valid = True
        for arm in ARM_ORDER:
            angles, scans, last_update = read_state(states[arm])
            fresh = last_update and time.monotonic() - last_update < 0.5
            if angles is None or not fresh:
                all_valid = False
                continue
            if scans != last_scan[arm]:
                samples[arm].append(angles)
                last_scan[arm] = scans

        if all_valid:
            valid_since = valid_since or time.monotonic()
            if time.monotonic() - valid_since >= sample_seconds:
                zero = {
                    arm: np.median(np.stack(samples[arm]), axis=0)
                    for arm in ARM_ORDER
                }
                for arm in ARM_ORDER:
                    formatted = ", ".join(f"{value:.1f}" for value in zero[arm])
                    print(f"{arm.capitalize()} zero (deg): [{formatted}]")
                return zero
        else:
            valid_since = None
        time.sleep(0.01)

    raise TimeoutError("Timed out waiting for 8 valid channels from both leader arms")


def map_targets(
    states: dict[str, ArmState],
    zero: dict[str, np.ndarray],
    mapping: dict[str, dict[str, np.ndarray]],
    ctrl_ranges: dict[str, np.ndarray],
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
        delta = np.radians(angles - zero[arm])
        delta = settings["sign"] * settings["scale"] * delta
        delta = np.clip(delta, -settings["max_delta"], settings["max_delta"])
        target = settings["home"] + delta
        targets[arm] = np.clip(target, ctrl_ranges[arm][:, 0], ctrl_ranges[arm][:, 1])
    return targets, oldest_age


def initialize_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mapping: dict[str, dict[str, np.ndarray]],
    actuator_ids: dict[str, np.ndarray],
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    ctrl_ranges: dict[str, np.ndarray] = {}
    homes = []
    mujoco.mj_resetData(model, data)

    for arm in ARM_ORDER:
        ids = actuator_ids[arm]
        ranges = model.actuator_ctrlrange[ids].copy()
        ctrl_ranges[arm] = ranges
        home = np.clip(mapping[arm]["home"], ranges[:, 0], ranges[:, 1])
        homes.append(home)
        data.ctrl[ids] = home
        for actuator_id, value in zip(ids, home, strict=True):
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            qpos_index = int(model.jnt_qposadr[joint_id])
            data.qpos[qpos_index] = value

    # Keep each unactuated second finger consistent with its equality-coupled mate.
    for arm in ARM_ORDER:
        first = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{arm}_finger_joint1"
        )
        second = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{arm}_finger_joint2"
        )
        data.qpos[model.jnt_qposadr[second]] = data.qpos[model.jnt_qposadr[first]]

    mujoco.mj_forward(model, data)
    return ctrl_ranges, np.concatenate(homes)


def main() -> int:
    args = parse_args()
    if args.left_port == args.right_port:
        raise SystemExit("Left and right serial ports must be different")
    if args.calibration_seconds <= 0 or args.calibration_timeout <= 0:
        raise SystemExit("Calibration timing values must be positive")
    if args.signal_timeout <= 0 or args.filter_tau < 0 or args.max_speed <= 0:
        raise SystemExit("Signal and filter timing values are invalid")

    mapping = load_mapping(args.mapping.resolve())
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    configure_lighting(model)
    data = mujoco.MjData(model)
    ids = actuator_indices(model)
    ctrl_ranges, filtered = initialize_pose(model, data, mapping, ids)

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
        print("Calibration complete. Move the leader arms to drive the simulation.")

        viewer_context = (
            nullcontext(None)
            if args.headless
            else mujoco.viewer.launch_passive(model, data)
        )
        started = time.monotonic()
        last_loop = started
        last_status = started
        signal_ok = True

        with viewer_context as viewer:
            while viewer is None or viewer.is_running():
                loop_started = time.monotonic()
                dt = max(loop_started - last_loop, model.opt.timestep)
                last_loop = loop_started

                targets, signal_age = map_targets(
                    states, zero, mapping, ctrl_ranges, args.signal_timeout
                )
                if targets is not None:
                    desired = np.concatenate([targets[arm] for arm in ARM_ORDER])
                    alpha = 1.0 if args.filter_tau == 0 else 1.0 - math.exp(-dt / args.filter_tau)
                    filtered_target = filtered + alpha * (desired - filtered)
                    max_step = args.max_speed * dt
                    filtered += np.clip(filtered_target - filtered, -max_step, max_step)
                    signal_ok = True
                else:
                    signal_ok = False

                for arm, values in zip(
                    ARM_ORDER,
                    np.split(filtered, len(ARM_ORDER)),
                    strict=True,
                ):
                    data.ctrl[ids[arm]] = values

                mujoco.mj_step(model, data)
                if viewer is not None:
                    viewer.sync()

                if loop_started - last_status >= 1.0:
                    state_text = "tracking" if signal_ok else "holding (serial timeout)"
                    print(f"Simulation: {state_text}, serial age={signal_age:.3f}s")
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
