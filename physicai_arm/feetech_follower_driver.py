#!/usr/bin/env python3
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from .feetech_common import FeetechStatePublisherBase, clamp
except ImportError:
    from feetech_common import FeetechStatePublisherBase, clamp


class FeetechFollowerDriverNode(FeetechStatePublisherBase):
    def __init__(self):
        super().__init__(
            node_name="feetech_follower_driver_node",
            default_arm_role="follower",
            default_joint_states_topic="/follower/joint_states",
        )

        self.declare_parameter("joint_targets_topic", "/follower/joint_targets")
        self.declare_parameter("safety_torque_topic", "/follower/safety/torque_enable")
        self.declare_parameter("write_speed", 32767)
        self.declare_parameter("write_acceleration", 50)
        self.declare_parameter("write_hz", 0.0)
        self.declare_parameter("command_timeout_ms", -1)
        self.declare_parameter("command_threshold_ticks", 1)
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("hold_last_target_on_timeout", True)
        # Absolute wrist_roll angle (rad) to snap to as soon as torque is enabled
        # (i.e. right at start-up). NaN -> disabled, i.e. stay wherever the arm
        # physically is. 1.5708 = +90 deg.
        self.declare_parameter("home_wrist_roll_rad", float("nan"))

        self.joint_targets_topic = self.get_parameter("joint_targets_topic").get_parameter_value().string_value
        self.safety_torque_topic = self.get_parameter("safety_torque_topic").get_parameter_value().string_value
        self.write_speed = int(self.get_parameter("write_speed").get_parameter_value().integer_value)
        self.write_acc = int(self.get_parameter("write_acceleration").get_parameter_value().integer_value)
        configured_write_hz = self.get_parameter("write_hz").get_parameter_value().double_value
        self.write_hz = configured_write_hz or self.cfg.control_hz
        configured_timeout_ms = int(self.get_parameter("command_timeout_ms").get_parameter_value().integer_value)
        self.command_timeout_ms = configured_timeout_ms if configured_timeout_ms >= 0 else self.cfg.command_timeout_ms
        self.command_threshold_ticks = max(
            1, int(self.get_parameter("command_threshold_ticks").get_parameter_value().integer_value)
        )
        self.heartbeat_hz = max(0.1, float(self.get_parameter("heartbeat_hz").get_parameter_value().double_value))
        self.hold_last_target_on_timeout = (
            self.get_parameter("hold_last_target_on_timeout").get_parameter_value().bool_value
        )
        home_wrist_roll = float(self.get_parameter("home_wrist_roll_rad").get_parameter_value().double_value)
        if math.isfinite(home_wrist_roll) and "wrist_roll" in self.cfg.joint_names:
            lo, hi = self.cfg.limit_rad["wrist_roll"]
            self._home_wrist_roll: Optional[float] = clamp(home_wrist_roll, lo, hi)
            self.get_logger().info(f"wrist_roll home-on-enable set to {self._home_wrist_roll:.4f} rad")
        else:
            self._home_wrist_roll = None

        current = self.get_measured_positions()
        self._torque_requested = True
        self._torque_enabled = False
        self._last_cmd_time = self.get_clock().now()
        self._last_write_wall = 0.0
        self._cmded = list(current)
        self._target_map: Dict[str, float] = {
            joint_name: pos for joint_name, pos in zip(self.cfg.joint_names, current)
        }
        self._last_written_ticks = [
            self.iface.rad_to_tick(joint_name, pos) for joint_name, pos in zip(self.cfg.joint_names, current)
        ]

        qos1 = QoSProfile(depth=1)
        self.sub = self.create_subscription(JointState, self.joint_targets_topic, self.on_target, qos1)
        self.safety_sub = self.create_subscription(Bool, self.safety_torque_topic, self.on_safety_torque_bool, 10)
        self._write_timer = self.create_timer(1.0 / max(1.0, self.write_hz), self.write_step)
        self.start_state_timer()

    def on_safety_torque_bool(self, msg: Bool) -> None:
        with self._state_lock:
            self._torque_requested = bool(msg.data)

    def on_target(self, msg: JointState) -> None:
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return

        updated = False
        with self._state_lock:
            for name, pos in zip(msg.name, msg.position):
                if name not in self._target_map:
                    continue
                lo, hi = self.cfg.limit_rad[name]
                self._target_map[name] = clamp(float(pos), lo, hi)
                updated = True
            if updated:
                self._last_cmd_time = self.get_clock().now()

    def _apply_torque_request(self) -> bool:
        with self._state_lock:
            requested = self._torque_requested
            torque_enabled = self._torque_enabled
            hold = list(self._measured)

        if requested == torque_enabled:
            return torque_enabled

        if not requested:
            with self._io_lock:
                self.iface.disable_all_servos()
            with self._state_lock:
                self._torque_enabled = False
            return False

        if self._home_wrist_roll is not None:
            wrist_roll_idx = self.cfg.joint_names.index("wrist_roll")
            hold[wrist_roll_idx] = self._home_wrist_roll

        with self._io_lock:
            self.iface.write_positions_rad(hold, speed=self.write_speed, acceleration=self.write_acc)

        now = self.get_clock().now()
        now_wall = time.monotonic()
        hold_ticks = [self.iface.rad_to_tick(joint_name, pos) for joint_name, pos in zip(self.cfg.joint_names, hold)]
        with self._state_lock:
            self._torque_enabled = True
            self._cmded = list(hold)
            self._target_map = {joint_name: pos for joint_name, pos in zip(self.cfg.joint_names, hold)}
            self._last_written_ticks = hold_ticks
            self._last_cmd_time = now
            self._last_write_wall = now_wall
        return True

    def write_step(self) -> None:
        try:
            torque_enabled = self._apply_torque_request()
        except Exception as exc:
            self.get_logger().warn(f"torque apply failed: {exc}")
            return

        if not torque_enabled:
            return

        now = self.get_clock().now()
        now_wall = time.monotonic()
        with self._state_lock:
            dt_ms = (now - self._last_cmd_time).nanoseconds / 1e6
            timed_out = dt_ms > float(self.command_timeout_ms)
            measured = list(self._measured)
            cmded = list(self._cmded)
            desired = [self._target_map[joint_name] for joint_name in self.cfg.joint_names]
            last_written_ticks = list(self._last_written_ticks)
            last_write_wall = self._last_write_wall

        if timed_out and self.hold_last_target_on_timeout:
            desired = list(cmded)
        elif timed_out:
            desired = list(measured)

        desired = [clamp(pos, *self.cfg.limit_rad[joint_name]) for joint_name, pos in zip(self.cfg.joint_names, desired)]
        desired_ticks = [
            self.iface.rad_to_tick(joint_name, pos) for joint_name, pos in zip(self.cfg.joint_names, desired)
        ]
        changed = any(
            abs(desired_tick - written_tick) >= self.command_threshold_ticks
            for desired_tick, written_tick in zip(desired_ticks, last_written_ticks)
        )
        heartbeat_due = (now_wall - last_write_wall) >= (1.0 / self.heartbeat_hz)

        if not changed and not heartbeat_due:
            return

        exact = [self.iface.tick_to_rad(joint_name, tick) for joint_name, tick in zip(self.cfg.joint_names, desired_ticks)]
        try:
            with self._io_lock:
                self.iface.write_positions_rad(exact, speed=self.write_speed, acceleration=self.write_acc)
        except Exception as exc:
            self.get_logger().warn(f"write_positions_rad failed: {exc}")
            return

        with self._state_lock:
            self._cmded = list(exact)
            self._last_written_ticks = list(desired_ticks)
            self._last_write_wall = now_wall


def main() -> None:
    rclpy.init()
    node = FeetechFollowerDriverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()