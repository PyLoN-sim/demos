"""Target translation force in N, composed with attitude torque in N*m."""

from typing import Tuple
from .core import Quaternion, Vector3, add, clamp_norm, quaternion_conjugate, rotate_vector, scale, subtract
from .attitude import body_attitude_torque

def target_force(position, orientation, velocity, desired_position, desired_velocity,
                 position_kp, velocity_kd, max_force, translation_enabled=True):
    """World-axis position/velocity PD, bounded and rotated into base_link."""
    if not translation_enabled:
        return (0.0, 0.0, 0.0)
    force = clamp_norm(add(scale(subtract(desired_position, position), position_kp),
                           scale(subtract(desired_velocity, velocity), velocity_kd)), max_force)
    return rotate_vector(quaternion_conjugate(orientation), force)

def body_wrench_for_setpoint(
    position: Vector3,
    orientation: Quaternion,
    linear_velocity: Vector3,
    angular_velocity_body: Vector3,
    desired_position: Vector3,
    desired_orientation: Quaternion,
    desired_linear_velocity: Vector3,
    desired_angular_velocity: Vector3,
    position_kp: float,
    velocity_kd: float,
    attitude_kp: float,
    angular_kd: float,
    max_force: float,
    max_torque: float,
    translation_enabled: bool,
    angular_rate_limit: float = 0.0,
) -> Tuple[Vector3, Vector3]:
    force = target_force(position, orientation, linear_velocity, desired_position,
                         desired_linear_velocity, position_kp, velocity_kd, max_force,
                         translation_enabled)
    torque = body_attitude_torque(orientation, angular_velocity_body, desired_orientation,
                                 desired_angular_velocity, attitude_kp, angular_kd,
                                 max_torque, angular_rate_limit)
    return force, torque
