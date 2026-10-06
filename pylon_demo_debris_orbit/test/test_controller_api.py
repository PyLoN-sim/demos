"""Current API guards and stop behavior; isolated from the KSP domain."""
import json
import math
import unittest
from unittest.mock import Mock

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from pylon_interfaces.msg import ControlAuthorityState, ControlSetpoint, VesselLifecycle, WrenchFeedback
    from debris_orbit.controller import ThrustController
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "source ROS and current PyLoN interfaces")
class ControllerApiTests(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=174)
        self.node = ThrustController(context=self.context)
        self.node.wrench_publisher = Mock()
        self.node.status_publisher = Mock()
        self.life = VesselLifecycle(vessel_id="chaser", generation=4, origin_sequence=7, state=1)
        self.node.receive_lifecycle(self.life)

    def tearDown(self):
        self.node.destroy_node()
        self.context.shutdown()

    def inputs(self, mode=None):
        n = self.node
        pose = PoseStamped()
        pose.header.frame_id = n.world_frame
        pose.header.stamp = n.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        twist = TwistStamped(header=pose.header)
        body = TwistStamped()
        body.header.frame_id = "base_link"
        body.header.stamp = pose.header.stamp
        target = ControlSetpoint(header=pose.header, mode=mode or ControlSetpoint.MODE_SIX_DOF)
        target.orientation.w = 1.0
        target.position.x = 2.0
        n.receive_pose(pose)
        n.receive_twist(twist)
        n.receive_body_twist(body)
        n.receive_setpoint(target)
        n.mode_started = n.get_clock().now() - Duration(seconds=4)

    def owned(self):
        # These producer IDs deliberately belong to a different PyLoN node.
        message = ControlAuthorityState(vessel_id="chaser", generation=4,
            controller_id="different_producer", lease_id="another_stream", state=1)
        self.node.receive_authority(message)
        return message

    def last_state(self):
        return json.loads(self.node.status_publisher.publish.call_args.args[0].data)["state"]

    def test_ksp_switch_enables_wrench_without_authority_commands(self):
        self.inputs()
        self.owned()
        self.node.control()
        command = self.node.wrench_publisher.publish.call_args.args[0]
        self.assertEqual(self.last_state(), "six_dof")
        self.assertEqual(command.vessel_id, "chaser")  # stale-vessel guard only
        self.assertEqual((command.controller_id, command.lease_id, command.sequence), ("", "", 0))
        self.assertEqual(command.header.frame_id, "base_link")
        self.assertGreater(command.wrench.force.x, 0)
        self.assertFalse(hasattr(self.node, "authority_publisher"))

    def test_stale_generation_cannot_grant_control(self):
        self.inputs()
        self.node.receive_authority(ControlAuthorityState(vessel_id="chaser", generation=3, state=1))
        self.node.control()
        self.node.wrench_publisher.publish.assert_not_called()
        self.assertEqual(self.last_state(), "waiting_for_ros2_control_on")

    def test_sensor_loss_zeros_and_latches_until_restart(self):
        self.inputs()
        self.owned()
        self.node.control()
        self.node.pose_received -= Duration(seconds=2)
        self.node.control()
        self.assertEqual(self.last_state(), "control_interrupted")
        command = self.node.wrench_publisher.publish.call_args.args[0]
        self.assertEqual(command.wrench.force.x, 0)
        self.inputs(mode=ControlSetpoint.MODE_IDLE)
        self.inputs()
        self.assertTrue(self.node.control_interrupted)
        self.assertIsNone(self.node.setpoint)

    def test_silent_authority_loss_is_a_latched_stop(self):
        self.inputs()
        self.owned()
        self.node.control()
        self.node.authority_received -= Duration(seconds=2)
        self.node.control()
        self.assertEqual(self.last_state(), "control_interrupted")
        self.assertFalse(self.node.commanding)

    def test_ksp_switch_off_latches_stop(self):
        self.inputs()
        message = self.owned()
        self.node.control()
        message.state = message.STATE_PLAYER
        self.node.receive_authority(message)
        self.node.control()
        self.assertTrue(self.node.control_interrupted)
        self.assertFalse(hasattr(self.node, "authority_publisher"))

    def test_idle_does_not_send_commands(self):
        self.node.receive_authority(ControlAuthorityState(vessel_id="chaser", generation=4, state=1))
        self.node.control()
        self.node.stop()
        self.node.wrench_publisher.publish.assert_not_called()

    def test_shutdown_while_switch_off_sends_no_command(self):
        self.inputs()
        self.node.control()
        self.node.stop()
        self.node.wrench_publisher.publish.assert_not_called()

    def test_continuous_limit_feedback_from_bridge_stream_starts_zero_cooldown(self):
        self.inputs()
        self.owned()
        self.node.receive_wrench_feedback(WrenchFeedback(vessel_id="chaser",
            controller_id="bridge_managed_stream", reason="continuous_actuation_limit"))
        self.node.control()
        self.assertEqual(self.last_state(), "safety_cooldown")
        self.assertEqual(self.node.wrench_publisher.publish.call_args.args[0].wrench.force.x, 0)

    def test_switch_to_same_vessel_new_generation_invalidates_old_state(self):
        self.inputs()
        self.owned()
        self.life.generation = 5
        self.node.receive_lifecycle(self.life)
        self.assertTrue(self.node.control_interrupted)
        self.assertIsNone(self.node.pose)
        self.assertFalse(self.node.commanding)

    def test_shutdown_zeros_wrench_and_preserves_ksp_switch(self):
        self.inputs()
        self.owned()
        self.node.control()
        self.node.stop()
        command = self.node.wrench_publisher.publish.call_args.args[0]
        self.assertEqual(command.wrench.force.x, 0.0)
        self.assertEqual(command.wrench.torque.z, 0.0)
        self.assertTrue(self.node.control_enabled)
        self.assertFalse(self.node.commanding)
        self.assertFalse(hasattr(self.node, "authority_publisher"))

    def test_nan_velocity_and_zero_quaternion_are_rejected(self):
        self.inputs()
        pose, twist = PoseStamped(), TwistStamped()
        pose.pose.orientation.w = 0.0
        pose.header.frame_id = twist.header.frame_id = self.node.world_frame
        twist.twist.linear.x = math.nan
        previous = self.node.pose, self.node.twist
        self.node.receive_pose(pose)
        self.node.receive_twist(twist)
        self.assertEqual((self.node.pose, self.node.twist), previous)
