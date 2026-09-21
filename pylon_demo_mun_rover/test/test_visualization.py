import unittest
from concurrent.futures import Future
import numpy as np
import xml.etree.ElementTree as ET
import rclpy
from geometry_msgs.msg import TransformStamped
from unittest.mock import Mock
from pylon_demo_mun_rover.visualization import Visualization,visual_model,proxy_boxes,PREFIX


def proxy():
    link='''<link name="pylon_12345678_link_0000"><inertial><mass value="1"/>
    <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="1 2 3"/></geometry></visual>
    <collision><geometry><box size="1 2 3"/></geometry></collision></link>'''
    return '<robot name="pylon_12345678_active_vessel">'+link+'</robot>'


class ModelTests(unittest.TestCase):
    def test_urdf_links_are_isolated_and_original_stays_unchanged(self):
        source=proxy();description,root,joints,count=visual_model(source)
        self.assertEqual(count,1);self.assertEqual(joints,[])
        self.assertEqual(root,'pylon_12345678_link_0000')
        self.assertEqual(ET.fromstring(description).find('link').attrib['name'],PREFIX+root)
        self.assertNotIn(PREFIX,source)
        boxes=proxy_boxes(source,root,joints)
        np.testing.assert_allclose(boxes[0][1],[.53,1.03,1.53])
        with self.assertRaises(ValueError):visual_model(source.replace('<box size="1 2 3"/>','<mesh filename="file:///tmp/x.stl"/>'))


class VisualizationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(domain_id=178);self.node=Visualization()
    def tearDown(self):
        self.node.destroy_node();rclpy.shutdown()
    def test_reset_rejects_pending_projection_and_clears_history(self):
        n=self.node;n.job_token=n.token;n.job_pose=np.eye(4)
        n.future=Future();n.future.set_result((np.array([[0.,0.,0.]]),np.array([[255,0,0]],np.uint8),np.array([1.])))
        n.reset_photos();n.photograph()
        self.assertEqual(n.photos.frames,0);self.assertEqual(len(n.photos.cloud()),0)
        self.assertIsNone(n.last_capture)

    def test_missing_model_clears_rviz_with_valid_geometry_free_urdf(self):
        n=self.node;n.description_pub=Mock();n.clear_model()
        robot=ET.fromstring(n.description_pub.publish.call_args.args[0].data)
        self.assertEqual(robot.tag,'robot');self.assertEqual(len(robot.findall('link')),1)
        self.assertEqual(len(robot.findall('.//visual')),0)
        self.assertEqual(n.model_links,0)
    def test_model_uses_only_relative_mount_and_estimator_branch(self):
        n=self.node;n.model_root='source_root';mount=TransformStamped();mount.transform.rotation.w=1.
        estimate=TransformStamped();estimate.transform.rotation.w=1.
        n.buffer=Mock();n.buffer.lookup_transform.side_effect=[mount,estimate];n.tf=Mock()
        n.model_transform()
        output=n.tf.sendTransform.call_args.args[0]
        self.assertEqual(output.header.frame_id,'pylon_rover_base_link')
        self.assertEqual(output.child_frame_id,PREFIX+'source_root')
        self.assertEqual([c.args[:2] for c in n.buffer.lookup_transform.call_args_list],
                         [('base_link','source_root'),('map','pylon_rover_base_link')])
    def test_packed_colour_cloud_serializes(self):
        from rclpy.serialization import serialize_message
        n=self.node;n.photo_pub=Mock()
        n.photos.add(np.array([[0.,0.,0.]]),np.array([[12,34,56]],np.uint8),np.array([1.]))
        n.publish_photos();msg=n.photo_pub.publish.call_args.args[0]
        self.assertEqual(msg.width,1);self.assertEqual(msg.point_step,16)
        self.assertTrue(serialize_message(msg))
