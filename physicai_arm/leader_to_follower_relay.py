#!/usr/bin/env python3
import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

try:
    from .feetech_common import load_joint_config
except ImportError:
    from feetech_common import load_joint_config

default_config_path = str(
    Path(get_package_share_directory("physicai_arm")) / "config" / "joints.yaml"
)

class LeaderToFollowerRelayNode(Node):
    def __init__(self):
        super().__init__("leader_to_follower_relay")
        self.declare_parameter("config_path", default_config_path)
        self.declare_parameter("follower_arm_role", "follower")
        self.declare_parameter("joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("input_topic", "/leader/joint_states")
        self.declare_parameter("output_topic", "/follower/joint_targets")

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        follower_arm_role = self.get_parameter("follower_arm_role").get_parameter_value().string_value or "follower"
        configured_joint_names = list(self.get_parameter("joint_names").get_parameter_value().string_array_value)
        if configured_joint_names:
            self.joint_names = configured_joint_names
        else:
            self.joint_names = load_joint_config(config_path, arm_role=follower_arm_role).joint_names

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._last_target: Optional[List[float]] = None
        self._last_invalid_warn_wall = 0.0
        self._last_missing_warn_wall = 0.0

        self.pub = self.create_publisher(JointState, self.output_topic, 10)
        self.sub = self.create_subscription(JointState, self.input_topic, self.on_leader_joint_state, 10)

    def _warn_every(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now_wall = time.monotonic()
        attr = "_last_invalid_warn_wall" if key == "invalid" else "_last_missing_warn_wall"
        last = getattr(self, attr)
        if (now_wall - last) >= period_sec:
            self.get_logger().warn(message)
            setattr(self, attr, now_wall)

    def on_leader_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            self._warn_every("invalid", "discarding invalid leader JointState: name/position mismatch or empty payload")
            return

        source = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        if self._last_target is None:
            missing = [joint_name for joint_name in self.joint_names if joint_name not in source]
            if missing:
                self._warn_every(
                    "missing",
                    f"discarding first leader JointState because required joints are missing: {missing}",
                )
                return
            target = [source[joint_name] for joint_name in self.joint_names]
        else:
            target = list(self._last_target)
            for index, joint_name in enumerate(self.joint_names):
                if joint_name in source:
                    target[index] = source[joint_name]

        if any(not math.isfinite(value) for value in target):
            self._warn_every("invalid", "discarding leader JointState containing non-finite joint positions")
            return

        self._last_target = list(target)

        out = JointState()
        out.header.stamp = (
            msg.header.stamp
            if (msg.header.stamp.sec or msg.header.stamp.nanosec)
            else self.get_clock().now().to_msg()
        )
        out.name = list(self.joint_names)
        out.position = list(target)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = LeaderToFollowerRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()