"""Read two 8-DOF servo leaders without commanding their motion.

The default setup is COM14 for the left leader and COM15 for the right
leader. All reported angles are raw servo angles in degrees; calibration and
OpenArm joint mapping belong in the simulation bridge built in the next step.
"""

from __future__ import annotations

import argparse
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Pattern

import serial


SERVO_COUNT = 8
PWM_PATTERN = re.compile(r"P(\d{4})")


def send_query(
    ser: serial.Serial,
    command: str,
    response_wait: float,
    expected_pattern: Pattern[str] | None = None,
) -> str:
    """Send a command and collect bytes until the expected reply or timeout."""
    ser.reset_input_buffer()
    ser.write(command.encode("ascii"))
    ser.flush()
    deadline = time.perf_counter() + response_wait
    response = bytearray()
    decoded = ""
    while time.perf_counter() < deadline:
        if ser.in_waiting:
            response.extend(ser.read_all())
            decoded = response.decode("ascii", errors="ignore")
            if expected_pattern is None or expected_pattern.search(decoded):
                break
        else:
            time.sleep(0.0005)
    return decoded


def pwm_to_angle(
    response: str,
    pwm_min: int = 500,
    pwm_max: int = 2500,
    angle_range: float = 270.0,
) -> float | None:
    match = PWM_PATTERN.search(response)
    if not match:
        return None

    pwm = int(match.group(1))
    if not pwm_min <= pwm <= pwm_max:
        return None
    return (pwm - pwm_min) / (pwm_max - pwm_min) * angle_range


@dataclass
class ArmState:
    angles: list[float | None] = field(
        default_factory=lambda: [None] * SERVO_COUNT
    )
    scans: int = 0
    errors: int = 0
    frequency_hz: float = 0.0
    last_update: float = 0.0
    torque_release_complete: bool = False
    torque_release_responses: list[str] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> tuple[list[float | None], int, float, float]:
        with self.lock:
            return (
                self.angles.copy(),
                self.errors,
                self.frequency_hz,
                self.last_update,
            )


class ArmReader(threading.Thread):
    def __init__(
        self,
        name: str,
        port: str,
        baudrate: int,
        startup_delay: float,
        response_wait: float,
        state: ArmState,
        stop_event: threading.Event,
        release_torque: bool = True,
        torque_release_settle: float = 1.0,
    ) -> None:
        super().__init__(name=f"{name}-reader", daemon=True)
        self.arm_name = name
        self.port = port
        self.baudrate = baudrate
        self.startup_delay = startup_delay
        self.response_wait = response_wait
        self.state = state
        self.stop_event = stop_event
        self.release_torque = release_torque
        self.torque_release_settle = torque_release_settle
        self.startup_error: Exception | None = None

    def run(self) -> None:
        try:
            with serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.response_wait,
                write_timeout=0.1,
            ) as ser:
                time.sleep(self.startup_delay)
                ser.reset_input_buffer()
                if self.release_torque:
                    responses = []
                    for servo_id in range(SERVO_COUNT):
                        response = send_query(
                            ser, f"#00{servo_id}PULK!", self.response_wait
                        )
                        responses.append(response.strip())
                    with self.state.lock:
                        self.state.torque_release_responses = responses
                        self.state.torque_release_complete = True
                    time.sleep(self.torque_release_settle)
                    ser.reset_input_buffer()
                    print(f"{self.arm_name}: torque release sent to 8 servos")
                previous_scan = time.perf_counter()
                while not self.stop_event.is_set():
                    angles: list[float | None] = []
                    scan_errors = 0

                    for servo_id in range(SERVO_COUNT):
                        if self.stop_event.is_set():
                            return
                        response = send_query(
                            ser,
                            f"#00{servo_id}PRAD!",
                            self.response_wait,
                            PWM_PATTERN,
                        )
                        angle = pwm_to_angle(response)
                        angles.append(angle)
                        if angle is None:
                            scan_errors += 1

                    now = time.perf_counter()
                    elapsed = now - previous_scan
                    previous_scan = now
                    with self.state.lock:
                        self.state.angles = angles
                        self.state.scans += 1
                        self.state.errors += scan_errors
                        self.state.frequency_hz = 1.0 / elapsed if elapsed else 0.0
                        self.state.last_update = time.monotonic()
        except (serial.SerialException, OSError) as exc:
            self.startup_error = exc
            self.stop_event.set()


def format_angles(angles: list[float | None]) -> str:
    return "[" + ", ".join(
        f"{angle:6.1f}" if angle is not None else "   nan"
        for angle in angles
    ) + "]"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read left and right 8-DOF servo leader angles."
    )
    parser.add_argument("--left-port", default="COM14")
    parser.add_argument("--right-port", default="COM15")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=0.2,
        help="seconds to let each serial adapter settle after opening",
    )
    parser.add_argument(
        "--response-wait",
        type=float,
        default=0.03,
        help="seconds to wait before reading each servo response",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=2.0,
        help="terminal updates per second",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop after this many seconds; default runs until Ctrl+C",
    )
    parser.add_argument(
        "--release-torque",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="send PULK to servos 0-7 before reading (default: enabled)",
    )
    parser.add_argument(
        "--torque-release-settle",
        type=float,
        default=1.0,
        help="seconds to discard asynchronous messages after PULK",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.left_port.upper() == args.right_port.upper():
        raise SystemExit("Left and right ports must be different.")
    if (
        args.startup_delay < 0
        or args.response_wait <= 0
        or args.print_rate <= 0
        or args.torque_release_settle < 0
    ):
        raise SystemExit("Timing values must be non-negative and rates positive.")

    stop_event = threading.Event()
    states = {"Left": ArmState(), "Right": ArmState()}
    readers = [
        ArmReader(
            "Left",
            args.left_port,
            args.baudrate,
            args.startup_delay,
            args.response_wait,
            states["Left"],
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
            states["Right"],
            stop_event,
            args.release_torque,
            args.torque_release_settle,
        ),
    ]

    print(
        f"left={args.left_port}, right={args.right_port}, baud={args.baudrate}, "
        f"release_torque={args.release_torque}"
    )
    print("No position commands will be sent. Press Ctrl+C to stop.")
    for reader in readers:
        reader.start()

    started = time.monotonic()
    try:
        while not stop_event.wait(1.0 / args.print_rate):
            for name in ("Left", "Right"):
                angles, errors, frequency, last_update = states[name].snapshot()
                valid = sum(angle is not None for angle in angles)
                age = time.monotonic() - last_update if last_update else float("inf")
                print(
                    f"{name:>5} {format_angles(angles)} "
                    f"valid={valid}/8 rate={frequency:4.1f}Hz "
                    f"errors={errors} age={age:4.2f}s"
                )
            print()

            if args.duration is not None and time.monotonic() - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for reader in readers:
            reader.join(timeout=1.0)

    failures = [
        f"{reader.arm_name} ({reader.port}): {reader.startup_error}"
        for reader in readers
        if reader.startup_error is not None
    ]
    if failures:
        print("Serial reader stopped with an error:")
        for failure in failures:
            print(f"  {failure}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
