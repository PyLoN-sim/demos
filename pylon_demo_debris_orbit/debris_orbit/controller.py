"""ROS adapter for source-tree force/attitude modules and KSP-enabled active-vessel control."""

import json
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from pylon_interfaces.msg import (
    BodyWrenchCommand,
    ControlAuthorityState,
    ControlSetpoint,
    VesselLifecycle,
    WrenchFeedback,
)
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String

from .thrust import body_wrench_for_setpoint
from .attitude import body_detumble_torque, rate_guard_body_torque
from .core import Quaternion, Vector3, add, linear_ramp_fraction, scale


class ThrustController(Node):
    """Convert coherent world-frame setpoints into body-frame wrench commands."""

    def __init__(self, **kwargs) -> None:
        super().__init__("pylon_demo_debris_orbit_controller", **kwargs)
        self._declare_parameters()
        prefix = "/" + self._string("vessel_topic_prefix").strip("/")
        pose_topic = self._string("pose_topic") or f"{prefix}/demos/debris_orbit/demo_vehicle/navigation/pose"
        twist_topic = self._string("twist_topic") or f"{prefix}/demos/debris_orbit/demo_vehicle/navigation/twist"
        body_twist_topic = self._string("body_twist_topic") or f"{prefix}/demos/debris_orbit/demo_vehicle/navigation/twist_body"
        setpoint_topic = self._string("setpoint_topic") or f"{prefix}/control/setpoint"
        wrench_topic = self._string("wrench_command_topic") or f"{prefix}/control/wrench_command"
        authority_state_topic = self._string("authority_state_topic") or f"{prefix}/control/authority/state"
        lifecycle_topic = self._string("vessel_lifecycle_topic") or f"{prefix}/lifecycle"
        feedback_topic = self._string("wrench_feedback_topic") or f"{prefix}/control/wrench_feedback"
        status_topic = self._string("controller_status_topic") or f"{prefix}/control/controller_status"

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.position_kp = self._positive("position_kp")
        self.velocity_kd = self._positive("velocity_kd")
        self.attitude_kp = self._positive("attitude_kp")
        self.angular_kd = self._positive("angular_kd")
        self.max_force = self._positive("max_force")
        self.max_torque = self._positive("max_torque")
        self.attitude_hold_max_torque = self._positive("attitude_hold_max_torque")
        self.attitude_hold_rate_limit = math.radians(
            self._positive("attitude_hold_rate_limit_deg_s")
        )
        self.detumble_kd = self._positive("detumble_kd")
        self.detumble_max_torque = self._positive("detumble_max_torque")
        self.command_ramp_sec = self._positive("command_ramp_sec")
        self.command_timeout_sec = self._positive("command_timeout_sec")
        self.authority_timeout = Duration(seconds=self._positive("authority_timeout_sec"))
        self.authority_received = None
        self.commanding = False
        self.stopping = False
        if not 0.05 <= self.command_timeout_sec <= 10.0:
            raise ValueError("command_timeout_sec must be within the PyLoN API range 0.05..10")
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.setpoint_timeout = Duration(seconds=self._positive("setpoint_timeout_sec"))
        self.extrapolate_setpoint = bool(self.get_parameter("extrapolate_setpoint").value)
        self.safety_cooldown = Duration(
            seconds=self._positive("control_safety_cooldown_sec")
        )

        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.body_twist: Optional[TwistStamped] = None
        self.setpoint: Optional[ControlSetpoint] = None
        self.pose_received: Optional[Time] = None
        self.twist_received: Optional[Time] = None
        self.body_twist_received: Optional[Time] = None
        self.setpoint_received: Optional[Time] = None
        self.active_mode = ControlSetpoint.MODE_IDLE
        self.mode_started: Optional[Time] = None
        self.safety_cooldown_until: Optional[Time] = None
        self.control_interrupted = False
        self.last_status = ""
        self.vessel_id = ""
        self.generation = None
        self.control_enabled = False

        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.wrench_publisher = self.create_publisher(BodyWrenchCommand, wrench_topic, command_qos)
        self.status_publisher = self.create_publisher(String, status_topic, status_qos)
        self.create_subscription(PoseStamped, pose_topic, self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, twist_topic, self.receive_twist, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, body_twist_topic, self.receive_body_twist, qos_profile_sensor_data
        )
        self.create_subscription(ControlSetpoint, setpoint_topic, self.receive_setpoint, 10)
        self.create_subscription(
            ControlAuthorityState, authority_state_topic, self.receive_authority, state_qos
        )
        self.create_subscription(
            VesselLifecycle, lifecycle_topic, self.receive_lifecycle, state_qos
        )
        self.create_subscription(
            WrenchFeedback, feedback_topic, self.receive_wrench_feedback, command_qos
        )
        self.create_timer(1.0 / self._positive("control_rate_hz"), self.control)
        self.get_logger().info(
            f"KSP ROS2 control setpoint={setpoint_topic} "
            f"wrench={wrench_topic} lifecycle={lifecycle_topic}"
        )

    def _declare_parameters(self) -> None:
        values = {
            "vessel_topic_prefix": "/ksp_vessel",
            "pose_topic": "",
            "twist_topic": "",
            "body_twist_topic": "",
            "setpoint_topic": "",
            "wrench_command_topic": "",
            "authority_state_topic": "",
            "vessel_lifecycle_topic": "",
            "wrench_feedback_topic": "",
            "controller_status_topic": "",
            "world_frame": "pylon_debris_inertial",
            "body_frame": "base_link",
            "authority_timeout_sec": 1.0,
            "command_timeout_sec": 0.25,
            "position_kp": 1000.0,
            "velocity_kd": 3000.0,
            "attitude_kp": 2000.0,
            "angular_kd": 4000.0,
            "max_force": 2000.0,
            "max_torque": 500.0,
            "attitude_hold_max_torque": 150.0,
            "attitude_hold_rate_limit_deg_s": 10.0,
            "detumble_kd": 1000.0,
            "detumble_max_torque": 500.0,
            "command_ramp_sec": 3.0,
            "state_timeout_sec": 0.5,
            "setpoint_timeout_sec": 0.5,
            "extrapolate_setpoint": False,
            "control_safety_cooldown_sec": 0.75,
            "control_rate_hz": 20.0,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"parameter {name} must be finite and greater than zero")
        return value

    def receive_lifecycle(self, message: VesselLifecycle) -> None:
        active = message.state in (
            VesselLifecycle.STATE_ACTIVE,
            VesselLifecycle.STATE_CHANGED,
        )
        vessel_id = message.vessel_id if active else ""
        if (vessel_id, message.generation) != (self.vessel_id, self.generation):
            if self.vessel_id:
                self.control_interrupted = True
            self.vessel_id, self.generation = vessel_id, message.generation
            self.pose = self.twist = self.body_twist = self.setpoint = None
            self.pose_received = self.twist_received = self.body_twist_received = self.setpoint_received = None
            self.authority_received = None
            self.control_enabled = False
            self.commanding = False

    def receive_authority(self, message: ControlAuthorityState) -> None:
        # Observe the KSP switch only; the demo never acquires/renews/releases it.
        if (message.vessel_id, message.generation) != (self.vessel_id, self.generation):
            return
        self.authority_received = self.get_clock().now()
        enabled = message.state == ControlAuthorityState.STATE_PYLON and not message.emergency_stop
        if self.commanding and not enabled:
            self.control_interrupted = True
            self.setpoint = None
        self.control_enabled = enabled

    def receive_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            return
        p, q = message.pose.position, message.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or sum(v*v for v in (q.x,q.y,q.z,q.w)) < 1e-12:
            return
        self.pose = message
        self.pose_received = self.get_clock().now()

    def receive_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            return
        if not self._finite_twist(message):
            return
        self.twist = message
        self.twist_received = self.get_clock().now()

    def receive_body_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.body_frame:
            return
        if not self._finite_twist(message):
            return
        self.body_twist = message
        self.body_twist_received = self.get_clock().now()

    def receive_setpoint(self, message: ControlSetpoint) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring setpoint in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        if self.control_interrupted or self.stopping:
            return
        now = self.get_clock().now()
        if message.mode != self.active_mode:
            self.active_mode = message.mode
            self.mode_started = now
        self.setpoint = message
        self.setpoint_received = now

    def receive_wrench_feedback(self, message: WrenchFeedback) -> None:
        if (
            message.vessel_id == self.vessel_id and self.control_enabled
            and "continuous_actuation_limit" in message.reason.split(",")
            and self.safety_cooldown_until is None
        ):
            self.safety_cooldown_until = self.get_clock().now() + self.safety_cooldown

    def control(self) -> None:
        now = self.get_clock().now()
        if self.stopping:
            return
        if self.commanding and (self.authority_received is None or
                now - self.authority_received > self.authority_timeout):
            self.control_interrupted = True
        if self.control_interrupted:
            self._zero_command(now)
            self._publish_status("control_interrupted")
            return
        if not self._setpoint_requests_control(now):
            expired = (self.setpoint is not None and self.setpoint.mode != ControlSetpoint.MODE_IDLE)
            was_commanding = self.commanding
            self._zero_command(now)
            if expired and was_commanding:
                self.control_interrupted = True
                self._publish_status("control_interrupted")
                return
            if self.setpoint is None:
                state = "waiting_for_setpoint"
            elif self.setpoint.mode == ControlSetpoint.MODE_IDLE:
                state = "idle"
            else:
                state = "failsafe_idle"
            self._publish_status(state)
            return
        if not self._inputs_are_fresh(now):
            self._zero_command(now)
            self.control_interrupted = True
            self.setpoint = None
            self.active_mode = ControlSetpoint.MODE_IDLE
            self.mode_started = None
            self._publish_status("control_interrupted")
            return
        if self.safety_cooldown_until is not None:
            if now < self.safety_cooldown_until:
                if self.control_enabled:
                    self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                self._publish_status("safety_cooldown")
                return
            self.safety_cooldown_until = None
            self.mode_started = now
        if (not self.control_enabled or self.authority_received is None
                or now - self.authority_received > self.authority_timeout):
            self._publish_status(
                "waiting_for_ros2_control_on",
                vessel_id=self.vessel_id,
            )
            return

        self.commanding = True
        assert (
            self.pose is not None
            and self.twist is not None
            and self.body_twist is not None
            and self.setpoint is not None
        )
        mode = self.setpoint.mode
        if mode == ControlSetpoint.MODE_IDLE:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("idle")
            return

        orientation = self._orientation()
        angular_velocity_body = self._angular_velocity_body()
        if mode == ControlSetpoint.MODE_DETUMBLE:
            force = (0.0, 0.0, 0.0)
            torque = body_detumble_torque(
                angular_velocity_body, self.detumble_kd, self.detumble_max_torque
            )
            state = "detumbling"
        elif mode in (ControlSetpoint.MODE_ATTITUDE_HOLD, ControlSetpoint.MODE_SIX_DOF):
            desired = self.setpoint
            desired_position = (desired.position.x, desired.position.y, desired.position.z)
            if self.extrapolate_setpoint:
                dt = (Time.from_msg(self.pose.header.stamp) - Time.from_msg(desired.header.stamp)).nanoseconds * 1.0e-9
                if abs(dt) > self.setpoint_timeout.nanoseconds * 1.0e-9:
                    self._zero_command(now)
                    self.control_interrupted = True
                    self._publish_status("control_interrupted", reason="setpoint_timestamp_mismatch")
                    return
                desired_position = add(desired_position, scale(
                    (desired.linear_velocity.x, desired.linear_velocity.y, desired.linear_velocity.z), dt))
            force, torque = body_wrench_for_setpoint(
                self._position(), orientation, self._linear_velocity(), angular_velocity_body,
                desired_position,
                (desired.orientation.x, desired.orientation.y, desired.orientation.z, desired.orientation.w),
                (desired.linear_velocity.x, desired.linear_velocity.y, desired.linear_velocity.z),
                (desired.angular_velocity.x, desired.angular_velocity.y, desired.angular_velocity.z),
                self.position_kp, self.velocity_kd, self.attitude_kp, self.angular_kd,
                self.max_force,
                self.max_torque if mode == ControlSetpoint.MODE_SIX_DOF else min(
                    self.max_torque, self.attitude_hold_max_torque
                ),
                mode == ControlSetpoint.MODE_SIX_DOF,
                self.attitude_hold_rate_limit,
            )
            torque = rate_guard_body_torque(
                torque,
                angular_velocity_body,
                self.attitude_hold_rate_limit,
                self.angular_kd,
                self.max_torque if mode == ControlSetpoint.MODE_SIX_DOF else min(
                    self.max_torque, self.attitude_hold_max_torque
                ),
            )
            state = "six_dof" if mode == ControlSetpoint.MODE_SIX_DOF else "attitude_hold"
        else:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._publish_status("invalid_mode", mode=int(mode))
            return

        ramp = linear_ramp_fraction(
            0.0 if self.mode_started is None else (now - self.mode_started).nanoseconds * 1.0e-9,
            self.command_ramp_sec,
        )
        self._publish_wrench(now, scale(force, ramp), scale(torque, ramp))
        self._publish_status(state, ramp=round(ramp, 2))

    @staticmethod
    def _finite_twist(message):
        v, w = message.twist.linear, message.twist.angular
        return all(math.isfinite(x) for x in (v.x,v.y,v.z,w.x,w.y,w.z))

    def _zero_command(self, now):
        if self.commanding and self.vessel_id:
            self._publish_wrench(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self.commanding = False

    def stop(self):
        self.stopping = True
        if rclpy.ok(context=self.context):
            self._zero_command(self.get_clock().now())

    def _inputs_are_fresh(self, now: Time) -> bool:
        return bool(
            self.pose is not None and self.twist is not None
            and self.body_twist is not None and self.setpoint is not None
            and self.pose_received is not None and now - self.pose_received <= self.state_timeout
            and self.twist_received is not None and now - self.twist_received <= self.state_timeout
            and self.body_twist_received is not None
            and now - self.body_twist_received <= self.state_timeout
            and self.setpoint_received is not None and now - self.setpoint_received <= self.setpoint_timeout
        )

    def _setpoint_requests_control(self, now: Time) -> bool:
        return bool(
            self.setpoint is not None
            and self.setpoint_received is not None
            and now - self.setpoint_received <= self.setpoint_timeout
            and self.setpoint.mode in (
                ControlSetpoint.MODE_DETUMBLE,
                ControlSetpoint.MODE_ATTITUDE_HOLD,
                ControlSetpoint.MODE_SIX_DOF,
            )
        )

    def _position(self) -> Vector3:
        assert self.pose is not None
        value = self.pose.pose.position
        return value.x, value.y, value.z

    def _orientation(self) -> Quaternion:
        assert self.pose is not None
        value = self.pose.pose.orientation
        return value.x, value.y, value.z, value.w

    def _linear_velocity(self) -> Vector3:
        assert self.twist is not None
        value = self.twist.twist.linear
        return value.x, value.y, value.z

    def _angular_velocity_body(self) -> Vector3:
        assert self.body_twist is not None
        value = self.body_twist.twist.angular
        return value.x, value.y, value.z

    def _publish_wrench(self, now: Time, force: Vector3, torque: Vector3) -> None:
        message = BodyWrenchCommand()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.body_frame
        message.vessel_id = self.vessel_id
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        message.timeout_sec = self.command_timeout_sec
        self.wrench_publisher.publish(message)

    def _publish_status(self, state: str, **extra: object) -> None:
        serialized = json.dumps({"state": state, **extra}, sort_keys=True)
        if serialized == self.last_status:
            return
        self.last_status = serialized
        self.status_publisher.publish(String(data=serialized))

    def destroy_node(self) -> bool:
        self.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrustController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
