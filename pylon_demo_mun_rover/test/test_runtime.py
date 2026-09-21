"""ROS adapter failure tests in a separate DDS domain; never send to KSP."""
import unittest
from unittest.mock import Mock
import time
import numpy as np
from concurrent.futures import Future
try:
    import rclpy
    from pylon_demo_mun_rover.node import Rover
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy=None


@unittest.skipIf(rclpy is None,'source ROS Jazzy and the built workspace')
class RuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=178)
        self.node=Rover()
        self.node.child_goal=Mock()
        self.node.active_goal=Mock()
    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()
    def test_fault_latches_and_rejects_late_velocity(self):
        n=self.node;n.stop('imu_timeout')
        msg=Twist();msg.linear.x=.5;n.command(msg)
        self.assertEqual(n.target,(0.,0.));self.assertEqual(n.command_time,0.)
        n.child_goal.cancel_goal_async.assert_called_once()
        n.stop('second_failure');self.assertEqual(n.fault,'imu_timeout')
    def test_cancel_uses_same_stop_boundary(self):
        self.node.cancel_goal(Mock())
        self.assertEqual(self.node.fault,'goal_cancelled')
        self.node.child_goal.cancel_goal_async.assert_called_once()
    def test_nonfinite_or_holonomic_command_stops(self):
        msg=Twist();msg.linear.y=0.1;self.node.command(msg)
        self.assertEqual(self.node.fault,'unsupported_velocity')
    def test_timeout_tick_sends_brake_and_releases(self):
        n=self.node;n.ready_reason=Mock(return_value='imu_timeout');n.send_wheels=Mock();n.send_authority=Mock()
        n.tick();n.send_wheels.assert_called_once_with(0.,0.,1.)
        self.assertEqual(n.fault,'imu_timeout')
    def test_reset_refused_during_goal(self):
        from std_srvs.srv import Trigger
        response=self.node.reset_service(Trigger.Request(),Trigger.Response())
        self.assertFalse(response.success)

    def test_geometry_parameters_wait_for_active_planner(self):
        from lifecycle_msgs.srv import GetState
        from lifecycle_msgs.msg import State
        from test_core import geometry
        n=self.node;n.geometry=geometry();n.goal_tolerances=(.6,.2)
        n.param_client=Mock();n.planner_state_client=Mock()
        n.planner_state_client.service_is_ready.return_value=True
        pending=Future();n.planner_state_client.call_async.return_value=pending
        n.configure();n.param_client.set_parameters.assert_not_called()
        response=GetState.Response();response.current_state.id=State.PRIMARY_STATE_INACTIVE
        pending.set_result(response);n.configure();n.param_client.set_parameters.assert_not_called()
        active=Future();n.planner_state_client.call_async.return_value=active
        n.configure();response=GetState.Response();response.current_state.id=State.PRIMARY_STATE_ACTIVE
        active.set_result(response);n.configure()
        self.assertTrue(n.planner_active)
        applied=n.param_client.set_parameters.call_args.args[0][0]
        self.assertEqual(applied.name,'GridBased.minimum_turning_radius')
        self.assertEqual(applied.value,n.geometry.min_radius)

    def test_lidar_auto_selection_switch_resets_and_ambiguous_sources_stop(self):
        n=self.node; n.active_goal=None
        a='/ksp_vessel/lidar_3d/rober_b/points'; b='/ksp_vessel/lidar_3d/other/points'
        n.get_topic_names_and_types=Mock(return_value=[(a,['sensor_msgs/msg/PointCloud2'])])
        n.count_publishers=Mock(return_value=1)
        before=n.reset_serial; n.select_lidar()
        self.assertEqual(n.lidar_topic,a);self.assertGreater(n.reset_serial,before)
        n.terrain.grid.fill(0); n.active_goal=Mock(); n.child_goal=Mock()
        n.get_topic_names_and_types.return_value=[(b,['sensor_msgs/msg/PointCloud2'])]
        n.select_lidar()
        self.assertEqual(n.fault,'lidar_source_changed')
        self.assertEqual(n.lidar_topic,b);self.assertTrue(np.all(n.terrain.grid==-1))
        n.get_topic_names_and_types.return_value += [(a,['sensor_msgs/msg/PointCloud2'])]
        n.select_lidar()
        self.assertEqual(n.lidar_topic,'');self.assertIsNone(n.lidar_subscription)
        self.assertIn('multiple_3d_lidars',n.lidar_selection_reason)
        n.count_publishers.return_value=0; n.select_lidar()
        self.assertEqual(n.lidar_selection_reason,'waiting_for_3d_lidar')

    def test_goal_brake_requires_position_and_heading_and_holds_late_twists(self):
        from nav2_msgs.action import NavigateToPose
        from test_core import geometry
        n=self.node; n.geometry=geometry(); n.goal_tolerances=(.6,.2)
        n.active_goal.request=NavigateToPose.Goal()
        pose=n.active_goal.request.pose.pose
        pose.position.x=-.8; pose.orientation.w=1.
        self.assertTrue(n.inside_goal())
        pose.orientation.z=np.sin(.2); pose.orientation.w=np.cos(.2)
        self.assertFalse(n.inside_goal())
        pose.orientation.z=0.; pose.orientation.w=1.; pose.position.x=0.
        self.assertFalse(n.inside_goal())
        pose.position.x=-.8
        n.configure=Mock(); n.ready_reason=Mock(return_value=''); n.send_wheels=Mock(); n.send_authority=Mock()
        n.command_time=time.monotonic(); n.target=(.03,0.)
        n.tick(); self.assertTrue(n.goal_braking)
        n.send_wheels.assert_called_with(0.,0.,1.)
        msg=Twist(); msg.linear.x=.5; n.command(msg); n.tick()
        n.send_wheels.assert_called_with(0.,0.,1.)
        n.active_goal.succeed.assert_not_called()

    def test_old_terrain_job_cannot_repopulate_reset_map(self):
        from pylon_demo_mun_rover.terrain import Terrain
        n=self.node; old=Terrain(); old.grid.fill(0); old.bootstrapped=True
        n.map_future=Future(); n.map_future.set_result(old)
        n.map_job=(n.reset_serial,n.get_clock().now().to_msg(),time.monotonic())
        n.reset_state(); n.finish_map()
        self.assertIsNone(n.map_future)
        self.assertFalse(n.terrain.bootstrapped)
        self.assertTrue(np.all(n.terrain.grid==-1))

    def test_terrain_job_does_not_modify_published_snapshot(self):
        from pylon_demo_mun_rover.terrain import Terrain
        from test_core import geometry
        old=Terrain(size=20.)
        points=np.array([[x,y,0.] for x in np.arange(-3,3,.2) for y in np.arange(-3,3,.2)])
        new=Rover.build_map(old,points,np.eye(4),geometry(),False)
        self.assertGreater(np.count_nonzero(new.grid==0),0)
        self.assertEqual(old.cells,{})
        self.assertTrue(np.all(old.grid==-1))

    def test_evaluation_realigns_after_manual_estimator_reset(self):
        from pylon_demo_mun_rover.evaluation import Evaluation
        from std_msgs.msg import String
        observer=Evaluation()
        try:
            observer.geometry(String(data='{"rear_x":-1.0,"estimator_generation":1}'))
            observer.alignment=np.eye(4); observer.previous=(1.,np.zeros(3))
            observer.geometry(String(data='{"rear_x":-1.0,"estimator_generation":1}'))
            self.assertIsNotNone(observer.alignment)
            observer.geometry(String(data='{"rear_x":-1.0,"estimator_generation":2}'))
            self.assertIsNone(observer.alignment)
            self.assertIsNone(observer.previous)
        finally: observer.destroy_node()

    def test_real_command_serialization_matches_generated_ros_types(self):
        from rclpy.serialization import serialize_message
        from pylon_vehicle_control.application.lease import LeaseAction
        from test_core import geometry
        n=self.node;n.authority_pub=Mock();n.wheel_pub=Mock()
        n.send_authority(LeaseAction('acquire','v','c','l'))
        self.assertTrue(serialize_message(n.authority_pub.publish.call_args.args[0]))
        n.geometry=geometry();n.lease.observe_vessel('v',True);n.lease.owned=True
        n.send_wheels(.2,.02,0.)
        self.assertEqual(n.wheel_pub.publish.call_count,4)
        for call in n.wheel_pub.publish.call_args_list:self.assertTrue(serialize_message(call.args[0]))
