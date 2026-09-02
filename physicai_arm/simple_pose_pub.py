import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SimplePoseStampedPub(Node):
    def __init__(self):
        super().__init__('simple_pose_stamped_pub')
        self.declare_parameter('topic', '/target_pose')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('x', 0.20)
        self.declare_parameter('y', 0.00)
        self.declare_parameter('z', 0.15)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter('topic').value), 10)
        rate = float(self.get_parameter('rate').value)
        if not rate > 0.0:
            self.get_logger().warn(f"invalid rate {rate}, falling back to 20.0 Hz")
            rate = 20.0
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.pose.position.x = float(self.get_parameter('x').value)
        msg.pose.position.y = float(self.get_parameter('y').value)
        msg.pose.position.z = float(self.get_parameter('z').value)
        msg.pose.orientation.x = float(self.get_parameter('qx').value)
        msg.pose.orientation.y = float(self.get_parameter('qy').value)
        msg.pose.orientation.z = float(self.get_parameter('qz').value)
        msg.pose.orientation.w = float(self.get_parameter('qw').value)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePoseStampedPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
