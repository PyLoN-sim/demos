"""Source-tree configuration and three-node executor; no ament lookup."""

import argparse
from pathlib import Path
import signal
import subprocess
import sys

import yaml

from .core import sanitize_ros_component

ROOT = Path(__file__).resolve().parents[1]
SECTIONS = {
    "recognition": "pylon_debris_target_estimator",
    "guidance": "pylon_demo_debris_orbit",
    "controller": "pylon_demo_debris_orbit_controller",
}


def _handle_termination(*_):
    raise KeyboardInterrupt


def parameter_sets(args):
    """Read the same three parameter sections as the original launch file."""
    with args.config.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("config must contain the three node parameter sections")
    values = {}
    for role, section in SECTIONS.items():
        block = document.get(section, {})
        if not isinstance(block, dict):
            raise ValueError(f"{section} must be a mapping")
        value = block.get("ros__parameters", {})
        if not isinstance(value, dict):
            raise ValueError(f"{section}.ros__parameters must be a mapping")
        values[role] = dict(value)
    guidance, recognition, controller = (values[k] for k in ("guidance", "recognition", "controller"))
    for option, name in (("instance", "demo_instance_id"), ("prefix", "vessel_topic_prefix"),
                         ("lidar_sensor_id", "lidar_sensor_id"), ("camera_sensor_id", "camera_sensor_id"),
                         ("orbit_radius", "orbit_radius"), ("lidar_frame", "lidar_frame")):
        value = getattr(args, option)
        if value is not None:
            guidance[name] = value
    if args.enabled is not None:
        guidance["enabled"] = args.enabled
    if args.output_directory is not None:
        guidance["output_directory"] = str(args.output_directory)
    prefix = "/" + str(guidance.get("vessel_topic_prefix", "/ksp_vessel")).strip("/")
    instance = str(guidance.get("demo_instance_id", "demo_vehicle"))
    token = sanitize_ros_component(instance, "demo_vehicle")
    namespace = f"{prefix}/demos/debris_orbit/{token}"
    for parameters in values.values():
        parameters["vessel_topic_prefix"] = prefix
        parameters["world_frame"] = "pylon_debris_inertial_" + token
    for key in ("demo_instance_id", "lidar_sensor_id", "orbit_radius", "orbit_plane_normal", "target_source", "body_frame"):
        if key in guidance:
            recognition[key] = guidance[key]
    target = guidance.get("target_topic") or namespace + "/target"
    recognition["target_topic"] = guidance["target_topic"] = target
    setpoint = guidance.get("setpoint_topic") or namespace + "/setpoint"
    controller["setpoint_topic"] = guidance["setpoint_topic"] = setpoint
    for name in ("pose", "twist"):
        topic = guidance.get(name + "_topic") or namespace + "/navigation/" + name
        guidance[name + "_topic"] = controller[name + "_topic"] = topic
    controller["body_twist_topic"] = namespace + "/navigation/twist_body"
    controller["controller_status_topic"] = namespace + "/controller_status"
    controller["body_frame"] = guidance.get("body_frame", "base_link")
    return values, prefix, token, namespace


def parser():
    result = argparse.ArgumentParser(description="LiDAR/IMU debris orbit demo, run directly with Python")
    result.add_argument("--config", type=Path, default=ROOT / "config/pylon_demo_debris_orbit.yaml")
    result.add_argument("--enabled", action=argparse.BooleanOptionalAction, default=None,
                        help="enable orbit control (default: false in supplied YAML)")
    result.add_argument("--node", choices=("all", *SECTIONS), default="all",
                        help="run all three nodes, or one module in a separate process")
    result.add_argument("--no-controller", action="store_true", help="estimation/guidance only")
    result.add_argument("--no-estimator", action="store_true", help="use externally published estimates")
    result.add_argument("--rviz", action="store_true")
    result.add_argument("--instance")
    result.add_argument("--prefix")
    result.add_argument("--lidar-sensor-id")
    result.add_argument("--camera-sensor-id")
    result.add_argument("--lidar-frame")
    result.add_argument("--orbit-radius", type=float)
    result.add_argument("--output-directory", type=Path)
    return result


def main():
    # --help works without ROS; pass any --ros-args through to rclpy.
    command_line = sys.argv[1:]
    split = command_line.index("--ros-args") if "--ros-args" in command_line else len(command_line)
    cli = parser()
    args = cli.parse_args(command_line[:split])
    try:
        parameters, prefix, token, namespace = parameter_sets(args)
    except (OSError, ValueError, yaml.YAMLError) as error:
        cli.error(str(error))

    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from pylon_interfaces.msg import ControlAuthorityState
        from .recognition import TargetEstimator
        from .guidance import DebrisOrbitNode
        from .controller import ThrustController
    except ImportError as error:
        cli.error(f"source ROS 2 Jazzy and the PyLoN interface workspace first: {error}")
    if "generation" not in ControlAuthorityState.get_fields_and_field_types():
        cli.error("pylon_interfaces is outdated: rebuild/source current PyLoN interfaces (authority generation is required)")

    rclpy.init(args=command_line[split:], signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    nodes = []
    viewer = None
    signal.signal(signal.SIGTERM, _handle_termination)
    try:
        for role, node_type in (("recognition", TargetEstimator), ("guidance", DebrisOrbitNode), ("controller", ThrustController)):
            if args.node not in ("all", role):
                continue
            if (role == "recognition" and args.no_estimator) or (role == "controller" and args.no_controller):
                continue
            node = node_type(parameter_overrides=[Parameter(key, value=value) for key, value in parameters[role].items()])
            nodes.append(node)
            executor.add_node(node)
        if args.rviz:
            command = ["rviz2", "-d", str(ROOT / "rviz/pylon_demo_debris_orbit.rviz"),
                       "-f", "pylon_debris_view_" + token, "--ros-args"]
            for name in ("markers", "path", "points", "target_points"):
                command += ["-r", f"/pylon_demo_debris_orbit_view/{name}:={namespace}/{name}"]
            viewer = subprocess.Popen(command)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for node in nodes:
            # Drain DDS while the ROS context is still alive. Normal Ctrl-C
            # first stops guidance, then clears Wrench and releases authority.
            if isinstance(node, DebrisOrbitNode):
                node.enabled = False
            if isinstance(node, ThrustController):
                node.stop()
        if rclpy.ok():
            import time
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.02)
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if viewer is not None:
            viewer.terminate()
            try:
                viewer.wait(timeout=2)
            except subprocess.TimeoutExpired:
                viewer.kill()
                viewer.wait()
        if rclpy.ok():
            rclpy.shutdown()
