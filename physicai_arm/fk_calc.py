import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from scipy.spatial.transform import Rotation


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy(v):
    return rz(v[2]) @ ry(v[1]) @ rx(v[0])


def rot(axis, q):
    x, y, z = np.array(axis, dtype=float) / np.linalg.norm(axis)
    c, s, C = np.cos(q), np.sin(q), 1.0 - np.cos(q)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def tf(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(xyz, dtype=float)
    return T


class PhysicAIArmFKNode(Node):
    def __init__(self):
        super().__init__('physicai_arm_fk_node')
        self.state = {}
        self.chain = []
        self.names = []
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(String, '/robot_description', self.urdf_cb, q)
        self.create_subscription(JointState, '/follower/joint_states', self.js_cb, 50)
        self.pub = self.create_publisher(PoseStamped, '/ee_pose', 50)

    def urdf_cb(self, msg):
        root = ET.fromstring(msg.data)
        child_to_joint = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            a = j.find('axis')
            xyz = [0.0, 0.0, 0.0] if o is None else [float(v) for v in o.attrib.get('xyz', '0 0 0').split()]
            rr = [0.0, 0.0, 0.0] if o is None else [float(v) for v in o.attrib.get('rpy', '0 0 0').split()]
            axis = [1.0, 0.0, 0.0] if a is None else [float(v) for v in a.attrib.get('xyz', '1 0 0').split()]
            child = j.find('child').attrib['link']
            child_to_joint[child] = {
                'name': j.attrib['name'],
                'type': j.attrib['type'],
                'parent': j.find('parent').attrib['link'],
                'child': child,
                'xyz': xyz,
                'rpy': rr,
                'axis': axis,
            }
        chain = []
        child = 'gripper_frame_link'
        while child != 'base_link':
            if child not in child_to_joint:
                self.get_logger().warning('gripper_frame_link chain not found in robot_description')
                return
            chain.insert(0, child_to_joint[child])
            child = child_to_joint[child]['parent']
        self.chain = chain
        self.names = [j['name'] for j in chain if j['type'] in ('revolute', 'continuous', 'prismatic')]
        self.get_logger().info('robot_description loaded')

    def fk(self):
        T = np.eye(4)
        for j in self.chain:
            T = T @ tf(j['xyz'], rpy(j['rpy']))
            if j['type'] in ('revolute', 'continuous'):
                T = T @ tf([0.0, 0.0, 0.0], rot(j['axis'], self.state[j['name']]))
            elif j['type'] == 'prismatic':
                T = T @ tf(np.array(j['axis']) * self.state[j['name']], np.eye(3))
        return T

    def js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.state[n] = p
        if not self.chain or not all(n in self.state for n in self.names):
            return
        T = self.fk()
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'
        p.pose.position.x = float(T[0, 3])
        p.pose.position.y = float(T[1, 3])
        p.pose.position.z = float(T[2, 3])
        q = Rotation.from_matrix(T[:3,:3]).as_quat()
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicAIArmFKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
