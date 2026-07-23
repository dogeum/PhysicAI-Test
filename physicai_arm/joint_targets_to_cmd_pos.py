#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


@dataclass(frozen=True)
class JointSpec:
    ros_topic: str
    lower: float
    upper: float


class JointTargetsToCmdPos(Node):
    def __init__(self) -> None:
        super().__init__("joint_targets_to_cmd_pos")

        self.declare_parameter("input_topic", "/follower/joint_targets")
        self.declare_parameter(
            "joint_order",
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        )
        self.declare_parameter("gripper_is_percent", False)

        self._joint_specs: Dict[str, JointSpec] = {
            "shoulder_pan": JointSpec("/sim/shoulder_pan/cmd_pos", -1.91986, 1.91986),
            "shoulder_lift": JointSpec("/sim/shoulder_lift/cmd_pos", -1.74533, 1.74533),
            "elbow_flex": JointSpec("/sim/elbow_flex/cmd_pos", -1.69, 1.69),
            "wrist_flex": JointSpec("/sim/wrist_flex/cmd_pos", -1.65806, 1.65806),
            "wrist_roll": JointSpec("/sim/wrist_roll/cmd_pos", -2.74385, 2.84121),
            "gripper": JointSpec("/sim/gripper/cmd_pos", -0.174533, 1.74533),
        }

        self._publishers = {
            name: self.create_publisher(Float64, spec.ros_topic, 10)
            for name, spec in self._joint_specs.items()
        }

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self._fallback_order: List[str] = list(
            self.get_parameter("joint_order").get_parameter_value().string_array_value
        )
        self._gripper_is_percent = (
            self.get_parameter("gripper_is_percent").get_parameter_value().bool_value
        )

        self._subscriber = self.create_subscription(
            JointState,
            input_topic,
            self._callback,
            10,
        )

        self.get_logger().info(f"Listening on {input_topic} and forwarding to per-joint cmd_pos topics.")

    def _callback(self, msg: JointState) -> None:
        pairs: List[Tuple[str, float]] = []

        if msg.name:
            if len(msg.name) != len(msg.position):
                self.get_logger().warn(
                    "Received JointState with mismatched name / position lengths. Ignoring message."
                )
                return
            pairs = list(zip(msg.name, msg.position))
        else:
            if len(msg.position) != len(self._fallback_order):
                self.get_logger().warn(
                    "Received unnamed JointState whose position length does not match joint_order."
                )
                return
            pairs = list(zip(self._fallback_order, msg.position))

        for joint_name, raw_value in pairs:
            spec = self._joint_specs.get(joint_name)
            if spec is None:
                continue

            value = raw_value
            if joint_name == "gripper" and self._gripper_is_percent:
                value = spec.lower + (spec.upper - spec.lower) * max(0.0, min(100.0, raw_value)) / 100.0

            clipped = max(spec.lower, min(spec.upper, value))
            if clipped != value:
                self.get_logger().debug(
                    f"Clipped {joint_name} from {value:.4f} to {clipped:.4f}."
                )

            out = Float64()
            out.data = clipped
            self._publishers[joint_name].publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTargetsToCmdPos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
