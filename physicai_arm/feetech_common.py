#!/usr/bin/env python3
import math
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _package_config_dir() -> Optional[str]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory("physicai_arm"), "config")
    except Exception:
        return None


def resolve_config_path(path: str) -> str:
    """Resolve a config path that may be empty, a bare filename, or a directory to a yaml file.

    Falls back to the installed ``physicai_arm/config`` share directory so nodes can be
    started with plain ``ros2 run`` (without a launch file supplying ``config_path``).
    """
    if not path:
        path = "joints.yaml"

    # A directory: look for joints.yaml inside it.
    if os.path.isdir(path):
        candidate = os.path.join(path, "joints.yaml")
        if os.path.isfile(candidate):
            return candidate
        raise FileNotFoundError(
            f"config_path is a directory and contains no joints.yaml: {path}"
        )

    if os.path.isfile(path):
        return path

    # Not found as given: try resolving a bare name against the package share dir.
    config_dir = _package_config_dir()
    if config_dir is not None:
        candidate = os.path.join(config_dir, os.path.basename(path))
        if os.path.isfile(candidate):
            return candidate

    raise FileNotFoundError(f"config file not found: {path}")


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


@dataclass
class JointConfig:
    joint_names: List[str]
    servo_id: Dict[str, int]
    sign: Dict[str, int]
    zero_offset_rad: Dict[str, float]
    limit_rad: Dict[str, Tuple[float, float]]
    max_vel_rad_s: Dict[str, float]
    publish_hz: float
    control_hz: float
    command_timeout_ms: int
    device: str
    baudrate: int
    ticks_per_rev: int
    center_tick: int
    servo_type: str


def _build_joint_config(raw: dict, device: str, baudrate: int) -> JointConfig:
    joint_names = list(raw["joint_names"])
    servo_id = {k: int(v) for k, v in raw["servo_id"].items()}
    sign = {k: int(v) for k, v in (raw.get("sign", {}) or {}).items()}
    zero_offset_rad = {k: float(v) for k, v in (raw.get("zero_offset_rad", {}) or {}).items()}
    limit_rad = {k: (float(v[0]), float(v[1])) for k, v in (raw.get("limit_rad", {}) or {}).items()}
    max_vel_rad_s = {k: float(v) for k, v in (raw.get("max_vel_rad_s", {}) or {}).items()}
    publish_hz = float(raw.get("publish_hz", 50.0))
    control_hz = float(raw.get("control_hz", 100.0))
    command_timeout_ms = int(raw.get("command_timeout_ms", 300))

    ft = (raw.get("feetech", {}) or {})
    ticks_per_rev = int(ft.get("ticks_per_rev", 4096))
    center_tick = int(ft.get("center_tick", ticks_per_rev // 2))
    servo_type = str(ft.get("servo_type", ft.get("model", "sts"))).lower()
    if "sts" in servo_type or "3215" in servo_type:
        servo_type = "sts"

    for joint_name in joint_names:
        if joint_name not in servo_id:
            raise ValueError(f"servo_id missing for joint '{joint_name}'")
        sign.setdefault(joint_name, 1)
        zero_offset_rad.setdefault(joint_name, 0.0)
        limit_rad.setdefault(joint_name, (-math.pi, math.pi))
        max_vel_rad_s.setdefault(joint_name, 1.0)

    return JointConfig(
        joint_names=joint_names,
        servo_id=servo_id,
        sign=sign,
        zero_offset_rad=zero_offset_rad,
        limit_rad=limit_rad,
        max_vel_rad_s=max_vel_rad_s,
        publish_hz=publish_hz,
        control_hz=control_hz,
        command_timeout_ms=command_timeout_ms,
        device=str(device),
        baudrate=int(baudrate),
        ticks_per_rev=ticks_per_rev,
        center_tick=center_tick,
        servo_type=servo_type,
    )


def load_joint_config(
    path: str,
    arm_role: str = "follower",
    device_override: Optional[str] = None,
    baudrate_override: Optional[int] = None,
) -> JointConfig:
    path = resolve_config_path(path)
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    if "joint_names" in raw and "servo_id" in raw:
        device = device_override or str(raw.get("device", "/dev/ttyUSB0"))
        baudrate = baudrate_override or int(raw.get("baudrate", 1000000))
        return _build_joint_config(raw, device=device, baudrate=baudrate)

    common = raw.get("common") or raw.get("joint_spec") or {}
    arms = raw.get("arms") or raw.get("hardware") or {}
    if not common:
        raise ValueError(
            "config file must contain legacy top-level joint fields or a 'common' section with shared joint spec"
        )
    arm_cfg = arms.get(arm_role, {}) or {}
    if not arm_cfg and device_override is None and baudrate_override is None:
        raise ValueError(f"arm role '{arm_role}' not found in config: {path}")

    device = device_override or arm_cfg.get("device") or common.get("device") or "/dev/ttyUSB0"
    baudrate = baudrate_override or arm_cfg.get("baudrate") or common.get("baudrate") or 1000000
    return _build_joint_config(common, device=str(device), baudrate=int(baudrate))


class FeetechInterface:
    def __init__(self, cfg: JointConfig, simulate: bool = False):
        self.cfg = cfg
        self.simulate = bool(simulate)
        self._sim_pos = [0.0 for _ in cfg.joint_names]
        self._controller = None
        self._servo_ids = [cfg.servo_id[joint_name] for joint_name in cfg.joint_names]
        if not self.simulate:
            self._init_real_bus()

    def _init_real_bus(self) -> None:
        try:
            from vassar_feetech_servo_sdk import ServoController
        except Exception as exc:
            raise RuntimeError(
                "vassar_feetech_servo_sdk import failed. Install: pip install vassar-feetech-servo-sdk"
            ) from exc

        self._controller = ServoController(
            servo_ids=self._servo_ids,
            servo_type=self.cfg.servo_type,
            port=self.cfg.device,
            baudrate=self.cfg.baudrate,
        )
        self._controller.connect()

    def close(self) -> None:
        if self._controller is not None:
            try:
                self._controller.disconnect()
            finally:
                self._controller = None

    def disable_all_servos(self) -> None:
        if self.simulate:
            return
        self._controller.disable_all_servos()

    def rad_to_tick(self, joint_name: str, rad: float) -> int:
        sign = self.cfg.sign[joint_name]
        zero = self.cfg.zero_offset_rad[joint_name]
        shifted = (rad - zero) * sign
        return int(round(self.cfg.center_tick + (shifted / (2.0 * math.pi)) * self.cfg.ticks_per_rev))

    def tick_to_rad(self, joint_name: str, tick: int) -> float:
        sign = self.cfg.sign[joint_name]
        zero = self.cfg.zero_offset_rad[joint_name]
        shifted = (tick - self.cfg.center_tick) * (2.0 * math.pi) / self.cfg.ticks_per_rev
        return float((shifted * sign) + zero)

    def read_positions_rad(self) -> List[float]:
        if self.simulate:
            return list(self._sim_pos)
        positions = self._controller.read_all_positions()
        output: List[float] = []
        for joint_name in self.cfg.joint_names:
            servo_id = self.cfg.servo_id[joint_name]
            if servo_id not in positions:
                raise RuntimeError(f"read_all_positions missing servo id {servo_id}")
            output.append(self.tick_to_rad(joint_name, int(positions[servo_id])))
        return output

    def write_positions_rad(self, positions: List[float], speed: int, acceleration: int) -> None:
        if self.simulate:
            self._sim_pos = list(positions)
            return
        command = {
            self.cfg.servo_id[joint_name]: self.rad_to_tick(joint_name, rad)
            for joint_name, rad in zip(self.cfg.joint_names, positions)
        }
        results = self._controller.write_position(command, speed=speed, acceleration=acceleration)
        if isinstance(results, dict):
            failures = [servo_id for servo_id, ok in results.items() if not ok]
            if failures:
                raise RuntimeError(f"write_position failed for servo ids: {failures}")


class FeetechStatePublisherBase(Node):
    def __init__(self, node_name: str, default_arm_role: str, default_joint_states_topic: str):
        super().__init__(node_name)
        self.declare_parameter("config_path", "joints.yaml")
        self.declare_parameter("arm_role", default_arm_role)
        self.declare_parameter("simulate", False)
        self.declare_parameter("device_override", "")
        self.declare_parameter("baudrate_override", 0)
        self.declare_parameter("joint_states_topic", default_joint_states_topic)
        self.declare_parameter("state_read_hz", 0.0)

        cfg_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.arm_role = self.get_parameter("arm_role").get_parameter_value().string_value or default_arm_role
        simulate = self.get_parameter("simulate").get_parameter_value().bool_value
        device_override_value = self.get_parameter("device_override").get_parameter_value().string_value.strip()
        baudrate_override_value = int(self.get_parameter("baudrate_override").get_parameter_value().integer_value)
        device_override = device_override_value or None
        baudrate_override = baudrate_override_value if baudrate_override_value > 0 else None

        self.cfg = load_joint_config(
            cfg_path,
            arm_role=self.arm_role,
            device_override=device_override,
            baudrate_override=baudrate_override,
        )
        self.iface = FeetechInterface(self.cfg, simulate=simulate)
        self.joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        configured_state_read_hz = self.get_parameter("state_read_hz").get_parameter_value().double_value
        self.state_read_hz = configured_state_read_hz or self.cfg.publish_hz

        self._state_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._state_timer = None

        try:
            with self._io_lock:
                current = self.iface.read_positions_rad()
        except Exception as exc:
            self.get_logger().warn(f"initial read_positions_rad failed: {exc}")
            current = [0.0 for _ in self.cfg.joint_names]

        self._measured = list(current)
        self.pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.get_logger().info(
            f"loaded {self.arm_role} config: device={self.cfg.device}, "
            f"baudrate={self.cfg.baudrate}, joint_states_topic={self.joint_states_topic}"
        )

    def start_state_timer(self) -> None:
        if self._state_timer is None:
            self._state_timer = self.create_timer(1.0 / max(1.0, self.state_read_hz), self.state_step)

    def get_measured_positions(self) -> List[float]:
        with self._state_lock:
            return list(self._measured)

    def state_step(self) -> None:
        try:
            with self._io_lock:
                measured = self.iface.read_positions_rad()
        except Exception as exc:
            self.get_logger().warn(f"read_positions_rad failed: {exc}")
            return

        with self._state_lock:
            self._measured = list(measured)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.cfg.joint_names)
        msg.position = list(measured)
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.iface.close()
        finally:
            super().destroy_node()