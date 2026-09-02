#!/usr/bin/env python3
import rclpy

try:
    from .feetech_common import FeetechStatePublisherBase
except ImportError:
    from feetech_common import FeetechStatePublisherBase


class FeetechLeaderStatePublisherNode(FeetechStatePublisherBase):
    def __init__(self):
        super().__init__(
            node_name="feetech_leader_state_publisher_node",
            default_arm_role="leader",
            default_joint_states_topic="/leader/joint_states",
        )
        self.start_state_timer()


def main() -> None:
    rclpy.init()
    node = FeetechLeaderStatePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()