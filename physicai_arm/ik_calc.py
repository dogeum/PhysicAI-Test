import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

CHAIN = [
    {'name': 'shoulder_pan', 'type': 'revolute', 'xyz': [0.0388353, -8.97657e-09, 0.0624], 'rpy': [3.14159, 4.18253e-17, -3.14159], 'lower': -1.91986, 'upper': 1.91986},
    {'name': 'shoulder_lift', 'type': 'revolute', 'xyz': [-0.0303992, -0.0182778, -0.0542], 'rpy': [-1.5708, -1.5708, 0.0], 'lower': -1.74533, 'upper': 1.74533},
    {'name': 'elbow_flex', 'type': 'revolute', 'xyz': [-0.11257, -0.028, 1.73763e-16], 'rpy': [-3.63608e-16, 8.74301e-16, 1.5708], 'lower': -1.69, 'upper': 1.69},
    {'name': 'wrist_flex', 'type': 'revolute', 'xyz': [-0.1349, 0.0052, 3.62355e-17], 'rpy': [4.02456e-15, 8.67362e-16, -1.5708], 'lower': -1.65806, 'upper': 1.65806},
    {'name': 'wrist_roll', 'type': 'revolute', 'xyz': [5.55112e-17, -0.0611, 0.0181], 'rpy': [1.5708, 0.0486795, 3.14159], 'lower': -2.74385, 'upper': 2.84121},
    {'name': 'gripper_frame_joint', 'type': 'fixed', 'xyz': [-0.0079, -0.000218121, -0.0981274], 'rpy': [0.0, 3.14159, 0.0]},
]

NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
LOWER = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385], dtype=float)
UPPER = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121], dtype=float)
PAN_IDX = 0
ROLL_IDX = 4
PAN_X = float(CHAIN[0]['xyz'][0])
PAN_Y = float(CHAIN[0]['xyz'][1])
TICK_RAD = 2.0 * math.pi / 4096.0


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


FIXED = []
REV = []
for j in CHAIN:
    T = np.eye(4, dtype=float)
    T[:3, :3] = rz(j['rpy'][2]) @ ry(j['rpy'][1]) @ rx(j['rpy'][0])
    T[:3, 3] = np.array(j['xyz'], dtype=float)
    FIXED.append(T)
    REV.append(j.get('type', 'fixed') == 'revolute')


def rz4(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4, dtype=float)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def fk_pos(q):
    T = np.eye(4, dtype=float)
    k = 0
    for A, rev in zip(FIXED, REV):
        T = T @ A
        if rev:
            T = T @ rz4(float(q[k]))
            k += 1
    return T[:3, 3].copy()


def quat_pitch(msg):
    x = float(msg.pose.orientation.x)
    y = float(msg.pose.orientation.y)
    z = float(msg.pose.orientation.z)
    w = float(msg.pose.orientation.w)
    v = 2.0 * (w * y - z * x)
    if v > 1.0:
        v = 1.0
    if v < -1.0:
        v = -1.0
    return math.asin(v)


class PhysicAIArmIKNode(Node):
    def __init__(self):
        super().__init__('physicai_arm_ik_node')
        self.state = {}
        self.last_names = []
        self.q_meas = None
        self.q_target = None
        self.target_pos = None
        self.target_pitch = None
        self.gripper_open = False
        self.target_seq = 0
        self.solved_seq = -1
        self.solved_roll = None
        self.declare_parameter('joint_topic', '/follower/joint_states')
        self.declare_parameter('target_topic', '/target_pose')
        self.declare_parameter('gripper_topic', '/gripper_open')
        self.declare_parameter('output_topic', '/follower/joint_targets')
        self.declare_parameter('publish_hz', 60.0)
        self.declare_parameter('iters', 4)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('max_dq', 0.20)
        self.declare_parameter('fallback_err', 0.006)
        self.declare_parameter('accept_err', 0.020)
        self.declare_parameter('target_eps_pos', 0.0005)
        self.declare_parameter('target_eps_pitch', 0.01)
        self.declare_parameter('roll_resolve_eps', 0.02)
        self.declare_parameter('depth', 1)
        self.declare_parameter('quantize_to_servo_tick', True)
        self.declare_parameter('gripper_open_pos', 0.80)
        self.declare_parameter('gripper_closed_pos', 0.00)
        depth = int(self.get_parameter('depth').value)
        self.create_subscription(JointState, str(self.get_parameter('joint_topic').value), self.js_cb, depth)
        self.create_subscription(PoseStamped, str(self.get_parameter('target_topic').value), self.target_cb, depth)
        self.create_subscription(Bool, str(self.get_parameter('gripper_topic').value), self.gripper_cb, depth)
        self.pub = self.create_publisher(JointState, str(self.get_parameter('output_topic').value), depth)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if not publish_hz > 0.0:
            self.get_logger().warn(f"invalid publish_hz {publish_hz}, falling back to 60.0 Hz")
            publish_hz = 60.0
        self.timer = self.create_timer(1.0 / publish_hz, self.tick)

    def js_cb(self, msg):
        self.last_names = list(msg.name)
        for n, p in zip(msg.name, msg.position):
            self.state[n] = float(p)
        if all(n in self.state for n in NAMES):
            self.q_meas = np.array([self.state[n] for n in NAMES], dtype=float)

    def target_cb(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        pitch = float(quat_pitch(msg))
        if self.target_pos is None:
            self.target_pos = pos
            self.target_pitch = pitch
            self.target_seq += 1
            return
        eps_p = float(self.get_parameter('target_eps_pos').value)
        eps_a = float(self.get_parameter('target_eps_pitch').value)
        if np.max(np.abs(pos - self.target_pos)) >= eps_p or abs(pitch - self.target_pitch) >= eps_a:
            self.target_pos = pos
            self.target_pitch = pitch
            self.target_seq += 1

    def gripper_cb(self, msg):
        self.gripper_open = bool(msg.data)

    def pan_for(self, p):
        return float(np.clip(-math.atan2(float(p[1]) - PAN_Y, float(p[0]) - PAN_X), LOWER[PAN_IDX], UPPER[PAN_IDX]))

    def pack(self, q1, q2, q3, pitch, q5):
        q4 = float(np.clip(float(pitch) - float(q2) - float(q3), LOWER[3], UPPER[3]))
        return np.array([
            float(np.clip(q1, LOWER[0], UPPER[0])),
            float(np.clip(q2, LOWER[1], UPPER[1])),
            float(np.clip(q3, LOWER[2], UPPER[2])),
            q4,
            float(np.clip(q5, LOWER[4], UPPER[4])),
        ], dtype=float)

    def solve_seed(self, target_pos, target_pitch, seed):
        q1 = self.pan_for(target_pos)
        q5 = float(seed[ROLL_IDX])
        q2 = float(seed[1])
        q3 = float(seed[2])
        lam2 = float(self.get_parameter('damping').value) ** 2
        iters = int(self.get_parameter('iters').value)
        max_dq = float(self.get_parameter('max_dq').value)
        best_q = self.pack(q1, q2, q3, target_pitch, q5)
        best_err = float(np.linalg.norm(target_pos - fk_pos(best_q)))
        h = 1e-4
        for _ in range(iters):
            q = self.pack(q1, q2, q3, target_pitch, q5)
            p = fk_pos(q)
            e = target_pos - p
            en = float(np.linalg.norm(e))
            if en < best_err:
                best_err = en
                best_q = q.copy()
            qh = self.pack(q1, q2 + h, q3, target_pitch, q5)
            j0 = (fk_pos(qh) - p) / h
            qh = self.pack(q1, q2, q3 + h, target_pitch, q5)
            j1 = (fk_pos(qh) - p) / h
            J = np.column_stack((j0, j1))
            try:
                dq = np.linalg.solve(J.T @ J + lam2 * np.eye(2, dtype=float), J.T @ e)
            except np.linalg.LinAlgError:
                break
            dq = np.clip(dq, -max_dq, max_dq)
            q2 = float(np.clip(q2 + float(dq[0]), LOWER[1], UPPER[1]))
            q3 = float(np.clip(q3 + float(dq[1]), LOWER[2], UPPER[2]))
        return best_q, best_err

    def solve_target(self, target_pos, target_pitch):
        q_ref = self.q_meas.copy()
        s0 = self.q_target.copy() if self.q_target is not None else q_ref.copy()
        best_q, best_err = self.solve_seed(target_pos, target_pitch, s0)
        if best_err > float(self.get_parameter('fallback_err').value):
            q1, e1 = self.solve_seed(target_pos, target_pitch, q_ref)
            if e1 < best_err:
                best_q, best_err = q1, e1
        if best_err > float(self.get_parameter('accept_err').value):
            best_q = self.pack(self.pan_for(target_pos), q_ref[1], q_ref[2], target_pitch, q_ref[4])
        return np.clip(best_q, LOWER, UPPER)

    def publish_target(self):
        if self.q_target is None or self.q_meas is None:
            return
        q = self.q_target.copy()
        if bool(self.get_parameter('quantize_to_servo_tick').value):
            q = np.round(q / TICK_RAD) * TICK_RAD
        gripper = float(self.get_parameter('gripper_open_pos').value) if self.gripper_open else float(self.get_parameter('gripper_closed_pos').value)
        qmap = {n: float(v) for n, v in zip(NAMES, q)}
        qmap['gripper'] = gripper
        names = list(self.last_names) if self.last_names else NAMES + ['gripper']
        if 'gripper' not in names:
            names = names + ['gripper']
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = [qmap[n] if n in qmap else float(self.state.get(n, 0.0)) for n in out.name]
        self.pub.publish(out)

    def tick(self):
        if self.q_meas is None or self.target_pos is None or self.target_pitch is None:
            return
        need_solve = self.q_target is None or self.target_seq != self.solved_seq
        if not need_solve and self.solved_roll is not None:
            if abs(float(self.q_meas[ROLL_IDX]) - float(self.solved_roll)) >= float(self.get_parameter('roll_resolve_eps').value):
                need_solve = True
        if need_solve:
            self.q_target = self.solve_target(self.target_pos, self.target_pitch)
            self.solved_seq = self.target_seq
            self.solved_roll = float(self.q_target[ROLL_IDX])
        self.publish_target()


def main(args=None):
    rclpy.init(args=args)
    node = PhysicAIArmIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
