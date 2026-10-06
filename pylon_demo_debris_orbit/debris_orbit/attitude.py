"""Sensor pointing, target attitude and body torque calculations; no ROS dependency."""

from .core import (
    Vector3, add, clamp_norm, cross, dot, norm, quaternion_between_vectors,
    quaternion_conjugate, quaternion_error_vector, quaternion_multiply,
    quaternion_normalize, rotate_vector, scale, search_direction, subtract,
)


def target_attitude(current, sensor_mount, direction):
    """Aim the real LiDAR +X axis with minimum rotation and preserve roll."""
    sensor_world = quaternion_multiply(current, sensor_mount)
    current_forward = rotate_vector(sensor_world, (1.0, 0.0, 0.0))
    correction = quaternion_between_vectors(current_forward, direction)
    return quaternion_normalize(quaternion_multiply(correction, current))


def target_angular_velocity(direction, relative_velocity, mount_velocity):
    """Line-of-sight feedforward in the startup IMU world axes, rad/s."""
    return scale(cross(direction, subtract(relative_velocity, mount_velocity)),
                 1.0 / max(dot(direction, direction), 0.01))


def body_attitude_torque(orientation, angular_velocity_body, desired_orientation,
                        desired_angular_velocity, attitude_kp, angular_kd,
                        max_torque, angular_rate_limit=0.0):
    world_to_body = quaternion_conjugate(orientation)
    attitude_error_body = rotate_vector(
        world_to_body,
        quaternion_error_vector(desired_orientation, orientation),
    )
    desired_angular_velocity_body = rotate_vector(
        world_to_body, desired_angular_velocity
    )
    # Treat attitude error as a requested body rate before closing the inner
    # angular-velocity loop.  Clamping that request prevents a large look-at
    # step from pinning torque at its limit until the craft has already spun
    # past the safety rate.  With no rate limit this is algebraically the same
    # PD law as attitude_kp * error + angular_kd * rate_error.
    requested_rate_body = add(
        desired_angular_velocity_body,
        scale(attitude_error_body, attitude_kp / angular_kd),
    )
    if angular_rate_limit > 0.0:
        requested_rate_body = clamp_norm(requested_rate_body, angular_rate_limit)
    torque_body = clamp_norm(
        scale(subtract(requested_rate_body, angular_velocity_body), angular_kd),
        max_torque,
    )
    return torque_body

def body_detumble_torque(
    angular_velocity_body: Vector3,
    damping: float,
    maximum: float,
) -> Vector3:
    return clamp_norm(scale(angular_velocity_body, -damping), maximum)


def rate_guard_body_torque(
    torque_body: Vector3,
    angular_velocity_body: Vector3,
    rate_limit: float,
    braking_gain: float,
    maximum: float,
) -> Vector3:
    rate = norm(angular_velocity_body)
    if rate <= rate_limit or rate <= 1.0e-9:
        return torque_body
    rate_axis = scale(angular_velocity_body, 1.0 / rate)
    current_parallel = dot(torque_body, rate_axis)
    required_braking = -min(maximum, braking_gain * (rate - rate_limit))
    if current_parallel > required_braking:
        torque_body = add(torque_body, scale(rate_axis, required_braking - current_parallel))
    return clamp_norm(torque_body, maximum)
