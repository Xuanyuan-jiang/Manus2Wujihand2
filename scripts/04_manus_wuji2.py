#!/usr/bin/env python3
"""Safely retarget ROS /hand_input data to a Wuji Hand 2.

The default mode is dry-run: it only prints the 20 retargeted joint angles and
never connects to, enables, or commands the physical hand.  Hardware control
requires both --control and an explicit confirmation phrase.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray
from wuji_sdk import HandModel, Handedness, JointCommand, RetargetSession, SdkManager


JOINT_COUNT = 20
INPUT_FLOAT_COUNT = 21 * 3
CONFIRMATION_PHRASE = "I_UNDERSTAND_THIS_MOVES_HARDWARE"

# Official Wuji Hand 2 Beta 2 joint ranges, in command order, converted to rad.
# Thumb S1..S4 followed by index/middle/ring/pinky S1..S4.
_THUMB_LIMITS_DEG = ((-68, 74), (-85, 40), (-60, 90), (-60, 90))
_FINGER_LIMITS_DEG = ((-60, 90), (-40, 40), (-60, 120), (-60, 90))
JOINT_LIMITS_RAD = np.deg2rad(
    np.asarray(_THUMB_LIMITS_DEG + _FINGER_LIMITS_DEG * 4, dtype=np.float64)
)


@dataclass(frozen=True)
class ControllerConfig:
    topic: str
    address: str
    control: bool
    rate_hz: float
    watchdog_s: float
    max_speed_rad_s: float
    warmup_frames: int
    kp: float
    kd: float
    effort_limit_a: float


def clamp_joint_positions(positions: np.ndarray) -> tuple[np.ndarray, int]:
    """Clamp a 20-vector to the documented Hand 2 position limits."""
    values = np.asarray(positions, dtype=np.float64)
    if values.shape != (JOINT_COUNT,) or not np.all(np.isfinite(values)):
        raise ValueError("retarget output must contain exactly 20 finite values")
    clipped = np.clip(values, JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1])
    return clipped, int(np.count_nonzero(np.abs(clipped - values) > 1e-9))


def rate_limit_positions(
    current: np.ndarray,
    target: np.ndarray,
    max_speed_rad_s: float,
    dt_s: float,
) -> np.ndarray:
    """Limit every joint's position change independently."""
    max_step = max_speed_rad_s * max(dt_s, 0.0)
    delta = np.clip(target - current, -max_step, max_step)
    return current + delta


def nid_to_command_index(nid: int) -> Optional[int]:
    """Map the SDK's 5-slot bus NID layout to the contiguous 20-axis order."""
    if nid <= 0:
        return None
    bus, node_index = divmod(nid - 1, 5)
    if bus >= 5 or node_index >= 4:
        return None
    return bus * 4 + node_index


def _recv_with_timeout(subscription, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = subscription.recv()
        if frame is not None:
            return frame
        time.sleep(0.002)
    raise TimeoutError("timed out waiting for Wuji Hand 2 data")


class ManusWuji2Controller(Node):
    """Consume MediaPipe landmarks and optionally command a Wuji Hand 2."""

    def __init__(self, config: ControllerConfig):
        super().__init__("manus_wuji2")
        self.config = config
        self.session = RetargetSession.for_hand(
            HandModel.WujiHand2, side=Handedness.Right
        )

        self._latest_keypoints: Optional[np.ndarray] = None
        self._latest_input_time: Optional[float] = None
        self._input_generation = 0
        self._processed_generation = 0
        self._valid_frames = 0
        self._last_report_time = 0.0
        self._last_command_time: Optional[float] = None
        self._last_command: Optional[np.ndarray] = None
        self._awaiting_post_enable_input = False
        self._post_enable_generation = 0
        self._post_enable_deadline = 0.0
        self._stale_reported = False
        self._clamp_total = 0
        self.shutdown_requested = False

        self._manager = None
        self._hand = None
        self._publisher = None
        self._diagnostics = None
        self._enabled = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Float32MultiArray, config.topic, self._input_callback, qos
        )
        self.create_timer(1.0 / config.rate_hz, self._control_tick)

        if config.control:
            self.get_logger().warning(
                "CONTROL mode requested; waiting for valid fresh input before enabling"
            )
        else:
            self.get_logger().info(
                "DRY-RUN mode: the physical hand will not be connected or commanded"
            )
        self.get_logger().info(
            f"subscribed to {config.topic} as BEST_EFFORT; expected 63 floats"
        )

    def _input_callback(self, message: Float32MultiArray) -> None:
        values = np.asarray(message.data, dtype=np.float32)
        if values.size != INPUT_FLOAT_COUNT:
            self.get_logger().error(
                f"rejecting /hand_input frame: expected 63 floats, got {values.size}"
            )
            return
        if not np.all(np.isfinite(values)):
            self.get_logger().error("rejecting /hand_input frame containing NaN/Inf")
            return

        keypoints = values.reshape(21, 3)
        hand_span_m = float(np.max(np.linalg.norm(keypoints - keypoints[0], axis=1)))
        if not 0.04 <= hand_span_m <= 0.35:
            self.get_logger().error(
                "rejecting implausible MediaPipe scale: "
                f"wrist-to-landmark span={hand_span_m:.4f} m"
            )
            return

        self._latest_keypoints = keypoints.copy()
        self._latest_input_time = time.monotonic()
        self._input_generation += 1
        self._stale_reported = False

    def _control_tick(self) -> None:
        if self.shutdown_requested:
            return

        try:
            now = time.monotonic()

            # SDK connect/enable is a blocking operation, so this single-threaded
            # ROS executor cannot service input callbacks while it runs.  After
            # enable, ignore the pre-connect timestamp and wait for one genuinely
            # new ROS frame before arming the normal 250 ms watchdog or sending
            # the first command.
            if self._awaiting_post_enable_input:
                got_new_frame = self._input_generation > self._post_enable_generation
                if got_new_frame and self._latest_input_time is not None:
                    age = now - self._latest_input_time
                    if age <= self.config.watchdog_s:
                        self._awaiting_post_enable_input = False
                        self.get_logger().warning(
                            "fresh post-enable /hand_input received; control output armed"
                        )
                    else:
                        got_new_frame = False
                if self._awaiting_post_enable_input:
                    if now > self._post_enable_deadline:
                        self._fail_stop(
                            "no fresh /hand_input frame arrived within 1.0s after enable"
                        )
                    return

            if self._latest_input_time is None:
                if now - self._last_report_time >= 2.0:
                    self.get_logger().info("waiting for the first valid /hand_input frame")
                    self._last_report_time = now
                return

            age = now - self._latest_input_time
            if age > self.config.watchdog_s:
                if self._enabled:
                    self._fail_stop(
                        f"input watchdog expired ({age:.3f}s > "
                        f"{self.config.watchdog_s:.3f}s)"
                    )
                elif not self._stale_reported:
                    self.session.reset()
                    self._valid_frames = 0
                    self._stale_reported = True
                    self.get_logger().warning(
                        f"input is stale ({age:.3f}s); retarget session reset"
                    )
                return

            if self._processed_generation == self._input_generation:
                self._check_diagnostics()
                return

            keypoints = self._latest_keypoints
            if keypoints is None:
                return
            raw_target = np.asarray(self.session.step(keypoints), dtype=np.float64)
            target, clamped = clamp_joint_positions(raw_target)
            self._clamp_total += clamped
            self._processed_generation = self._input_generation
            self._valid_frames += 1

            if now - self._last_report_time >= 1.0:
                mode = "control" if self.config.control else "dry-run"
                if self.config.control:
                    preview = ", ".join(f"{value:+.3f}" for value in target[:5])
                    qpos_text = f"qpos[0:5]=[{preview}]"
                else:
                    names = ("thumb", "index", "middle", "ring", "pinky")
                    groups = []
                    for finger_index, name in enumerate(names):
                        start = finger_index * 4
                        values = ",".join(
                            f"{value:+.3f}" for value in target[start : start + 4]
                        )
                        groups.append(f"{name}=[{values}]")
                    qpos_text = "qpos(rad) " + " ".join(groups)
                self.get_logger().info(
                    f"[{mode}] frames={self._valid_frames} age={age * 1000:.0f}ms "
                    f"{qpos_text} clamped_now={clamped}"
                )
                self._last_report_time = now

            if not self.config.control:
                return

            if not self._enabled:
                if self._valid_frames < self.config.warmup_frames:
                    return
                self._start_hardware()
                # Starting the SDK is intentionally blocking.  Do not send the
                # pre-connect target: return to the ROS executor and require a
                # new post-enable input frame before the first command.
                return

            assert self._last_command is not None
            assert self._last_command_time is not None
            dt_s = min(max(now - self._last_command_time, 0.0), 0.1)
            command = rate_limit_positions(
                self._last_command,
                target,
                self.config.max_speed_rad_s,
                dt_s,
            )
            self._publisher.send(
                [
                    JointCommand(position=float(p), velocity=0.0, effort=0.0)
                    for p in command
                ]
            )
            self._last_command = command
            self._last_command_time = now
            self._check_diagnostics()
        except Exception as exc:  # fail closed for SDK and retarget failures
            self._fail_stop(f"control loop error: {type(exc).__name__}: {exc}")

    def _read_joint_positions(self) -> np.ndarray:
        subscription = self._hand.joint_states().subscribe()
        try:
            frame = _recv_with_timeout(subscription, 2.0)
        finally:
            subscription.close()

        positions = np.full(JOINT_COUNT, np.nan, dtype=np.float64)
        for joint in frame.joints:
            index = nid_to_command_index(int(joint.nid))
            if index is not None:
                positions[index] = float(joint.position)
        if not np.all(np.isfinite(positions)):
            missing = np.flatnonzero(~np.isfinite(positions)).tolist()
            raise RuntimeError(f"joint_states missing command indices: {missing}")
        return positions

    def _read_and_check_diagnostics(self) -> None:
        subscription = self._hand.joint_diagnostics().subscribe()
        try:
            frame = _recv_with_timeout(subscription, 2.0)
        finally:
            subscription.close()
        if len(frame.joints) != JOINT_COUNT:
            raise RuntimeError(f"diagnostics contains {len(frame.joints)}/20 joints")
        errors = [
            (int(joint.nid), int(joint.error_code_current))
            for joint in frame.joints
            if int(joint.error_code_current) != 0
        ]
        if errors:
            raise RuntimeError(f"active joint errors: {errors}")

    def _wait_enabled(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self._diagnostics.recv()
            if frame is not None and len(frame.joints) == JOINT_COUNT:
                errors = [
                    (int(joint.nid), int(joint.error_code_current))
                    for joint in frame.joints
                    if int(joint.error_code_current) != 0
                ]
                if errors:
                    raise RuntimeError(f"active joint errors while enabling: {errors}")
                if all(joint.status_word.ext_state == 2 for joint in frame.joints):
                    return
            time.sleep(0.005)
        raise TimeoutError("not all 20 joints entered Enabled state within 5 seconds")

    def _start_hardware(self) -> None:
        self.get_logger().warning(
            f"connecting to physical Wuji Hand 2 at {self.config.address}"
        )
        self._manager = SdkManager.instance()
        self._hand = self._manager.connect(
            address=self.config.address, device_name="wuji_hand_2"
        )
        count = int(self._hand.online_joints_count().get())
        if count != JOINT_COUNT:
            raise RuntimeError(f"only {count}/20 Wuji Hand 2 joints are online")
        self._read_and_check_diagnostics()
        current = self._read_joint_positions()
        current, clamped = clamp_joint_positions(current)
        if clamped:
            raise RuntimeError(
                f"current hand pose has {clamped} joint(s) outside documented limits"
            )

        self._hand.effort_limit().set(self.config.effort_limit_a)
        self._hand.mit_params().set((self.config.kp, self.config.kd))
        self._diagnostics = self._hand.joint_diagnostics().subscribe()
        try:
            self._diagnostics.set_rate(int(round(self.config.rate_hz)))
        except Exception as exc:
            self.get_logger().warning(f"could not lower diagnostics rate: {exc}")

        self._hand.enable()
        # Mark this immediately: even if the following state wait fails, the
        # exception path must still call disable().
        self._enabled = True
        self._wait_enabled()
        self._publisher = self._hand.joint_command().publish()
        self._last_command = current
        self._last_command_time = time.monotonic()
        self.session.reset()
        self._post_enable_generation = self._input_generation
        self._post_enable_deadline = time.monotonic() + 1.0
        self._awaiting_post_enable_input = True
        self.get_logger().warning(
            f"HAND ENABLED: sn={self._hand.serial_number}, kp={self.config.kp}, "
            f"kd={self.config.kd}, effort_limit={self.config.effort_limit_a}A, "
            f"max_speed={self.config.max_speed_rad_s}rad/s"
        )

    def _check_diagnostics(self) -> None:
        if not self._enabled or self._diagnostics is None:
            return
        latest = None
        for _ in range(20):
            frame = self._diagnostics.recv()
            if frame is None:
                break
            latest = frame
        if latest is None:
            return
        errors = [
            (int(joint.nid), int(joint.error_code_current))
            for joint in latest.joints
            if int(joint.error_code_current) != 0
        ]
        if errors:
            raise RuntimeError(f"active joint errors: {errors}")
        if len(latest.joints) != JOINT_COUNT or not all(
            joint.status_word.ext_state == 2 for joint in latest.joints
        ):
            raise RuntimeError("one or more joints left Enabled state")

    def _fail_stop(self, reason: str) -> None:
        if self.shutdown_requested:
            return
        self.get_logger().error(f"FAIL-SAFE STOP: {reason}")
        self.shutdown_requested = True
        self.close()

    def close(self) -> None:
        """Close command resources, disable the hand, and disconnect."""
        if self._publisher is not None:
            try:
                self._publisher.close()
            except Exception:
                pass
            self._publisher = None
        if self._hand is not None and self._enabled:
            try:
                self._hand.disable()
                self.get_logger().warning("hand disabled")
            except Exception as exc:
                self.get_logger().error(f"failed to disable hand cleanly: {exc}")
            self._enabled = False
        if self._diagnostics is not None:
            try:
                self._diagnostics.close()
            except Exception:
                pass
            self._diagnostics = None
        if self._manager is not None:
            try:
                self._manager.disconnect_all()
            except Exception:
                pass
            self._manager = None
        self._hand = None


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return parsed


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Retarget /hand_input to Wuji Hand 2 (dry-run by default)."
    )
    parser.add_argument("--topic", default="/hand_input")
    parser.add_argument("--address", default="192.168.1.111:7447")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        dest="control",
        action="store_false",
        help="retarget and print only; never connect to the hand (default)",
    )
    mode.add_argument(
        "--control",
        action="store_true",
        help="enable and command the physical hand (requires --confirm)",
    )
    parser.set_defaults(control=False)
    parser.add_argument(
        "--confirm",
        default="",
        metavar="PHRASE",
        help=f"required with --control: {CONFIRMATION_PHRASE}",
    )
    parser.add_argument("--rate", type=_positive_float, default=100.0, metavar="HZ")
    parser.add_argument(
        "--watchdog", type=_positive_float, default=0.25, metavar="SECONDS"
    )
    parser.add_argument(
        "--max-speed", type=_positive_float, default=0.6, metavar="RAD_PER_SEC"
    )
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--kp", type=_positive_float, default=3.0)
    parser.add_argument("--kd", type=_positive_float, default=0.05)
    parser.add_argument(
        "--effort-limit", type=_positive_float, default=0.5, metavar="AMP"
    )
    args = parser.parse_args(argv)

    if args.control and args.confirm != CONFIRMATION_PHRASE:
        parser.error(
            "--control requires --confirm " + CONFIRMATION_PHRASE
        )
    if args.warmup_frames < 1:
        parser.error("--warmup-frames must be at least 1")
    if args.kp < 3.0:
        parser.error("--kp must be at least 3.0 per the official control guide")
    if not 0.01 <= args.kd <= 0.05:
        parser.error("--kd must be in the official 0.01..0.05 range")
    if args.effort_limit > 1.5:
        parser.error("--effort-limit must not exceed the recommended 1.5 A")
    if args.rate > 1000.0:
        parser.error("--rate must not exceed the SDK's 1000 Hz command limit")
    return args


def main() -> int:
    ros_argv = sys.argv
    cli_argv = remove_ros_args(ros_argv)[1:]
    args = parse_args(cli_argv)
    config = ControllerConfig(
        topic=args.topic,
        address=args.address,
        control=args.control,
        rate_hz=args.rate,
        watchdog_s=args.watchdog,
        max_speed_rad_s=args.max_speed,
        warmup_frames=args.warmup_frames,
        kp=args.kp,
        kd=args.kd,
        effort_limit_a=args.effort_limit,
    )

    rclpy.init(args=ros_argv)
    node = ManusWuji2Controller(config)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.shutdown_requested else 0


if __name__ == "__main__":
    raise SystemExit(main())
