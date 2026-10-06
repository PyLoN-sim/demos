"""Synthetic sensors -> recognition -> guidance -> Wrench over actual DDS."""
import time
import unittest
from unittest.mock import Mock

import numpy as np

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from pylon_interfaces.msg import BodyWrenchCommand, ControlAuthorityCommand, ControlAuthorityState, VesselLifecycle
    from debris_orbit.controller import ThrustController
    from debris_orbit.guidance import DebrisOrbitNode
    from debris_orbit.recognition import TargetEstimator
    from debris_orbit.runner import parameter_sets, parser
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "source ROS and current PyLoN interfaces")
class PipelineRosTests(unittest.TestCase):
    def test_source_config_sensor_to_wrench_and_shutdown(self):
        context = Context()
        rclpy.init(context=context, domain_id=175)
        executor = SingleThreadedExecutor(context=context)
        nodes = []
        try:
            values, _, _, _ = parameter_sets(parser().parse_args(["--enabled", "--instance", "pipeline_test"]))
            for role, cls in (("recognition", TargetEstimator), ("guidance", DebrisOrbitNode), ("controller", ThrustController)):
                node = cls(context=context, parameter_overrides=[Parameter(k, value=v) for k,v in values[role].items()])
                nodes.append(node)
                executor.add_node(node)
            recognition, guidance, controller = nodes
            fixture = Node("synthetic_debris_fixture", context=context)
            nodes.append(fixture)
            executor.add_node(fixture)
            durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            life_pub = fixture.create_publisher(VesselLifecycle, "/ksp_vessel/lifecycle", durable)
            state_pub = fixture.create_publisher(ControlAuthorityState, "/ksp_vessel/control/authority/state", durable)
            imu_pub = fixture.create_publisher(Imu, "/ksp_vessel/imu/data_raw", qos_profile_sensor_data)
            cloud_pub = fixture.create_publisher(PointCloud2, "/ksp_vessel/lidar_3d/front_lidar/points", qos_profile_sensor_data)
            commands, actions = [], []

            fixture.create_subscription(ControlAuthorityCommand, "/ksp_vessel/control/authority/command", actions.append, 10)
            fixture.create_subscription(BodyWrenchCommand, "/ksp_vessel/control/wrench_command", commands.append, 10)
            # Sensor mount input is supplied without depending on a KSP model.
            mount = TransformStamped()
            mount.transform.rotation.w = 1.0
            recognition.pipeline.buffer.lookup_transform = Mock(return_value=mount)
            guidance.tf_buffer.lookup_transform = Mock(return_value=mount)
            life = VesselLifecycle(vessel_id="synthetic_chaser", generation=7, origin_sequence=1, state=1)
            rng = np.random.default_rng(32)
            points = (rng.uniform(-0.3, 0.3, (100, 3)) + [15, 0, 0]).astype(np.float32)
            started = time.monotonic()
            last_cloud = last_life = -1.0
            while time.monotonic() - started < 3.0:
                elapsed = time.monotonic() - started
                if elapsed - last_life > 0.2:
                    life_pub.publish(life)
                    state_pub.publish(ControlAuthorityState(vessel_id="synthetic_chaser", generation=7, state=1))
                    last_life = elapsed
                imu = Imu(header=Header(stamp=fixture.get_clock().now().to_msg(), frame_id="base_link"))
                imu_pub.publish(imu)
                if elapsed - last_cloud > 0.1:
                    header = Header(stamp=imu.header.stamp, frame_id="synthetic_lidar")
                    cloud_pub.publish(create_cloud_xyz32(header, points))
                    last_cloud = elapsed
                executor.spin_once(timeout_sec=0.002)
                time.sleep(0.005)

            self.assertGreaterEqual(recognition.observations, 3)
            self.assertIsNotNone(guidance.orbit_started)
            self.assertFalse(controller.control_interrupted)
            self.assertEqual(actions, [])
            self.assertTrue(any(abs(cmd.wrench.force.y) > 1.0 for cmd in commands))
            self.assertTrue(all(cmd.controller_id == "" and cmd.sequence == 0 for cmd in commands))
            for node in nodes[:3]:
                subscriptions = node.get_subscriber_names_and_types_by_node(node.get_name(), node.get_namespace())
                self.assertFalse(any("/ground_truth/" in topic for topic, _ in subscriptions))

            guidance.enabled = False
            controller.stop()
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.01)
            self.assertEqual(actions, [])
            self.assertEqual(commands[-1].wrench.force.x, 0.0)
            self.assertEqual(commands[-1].wrench.force.y, 0.0)
            self.assertEqual(commands[-1].wrench.torque.z, 0.0)
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            context.shutdown()
