"""Independent RViz model and camera map; never publishes driving inputs."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from array import array
import json
import math
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String,Header
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from pylon_interfaces.msg import VesselLifecycle
from pylon_bridge.static_transforms import StaticTransformSnapshot
from pylon_bridge.vessel_model import _parse_proxy_urdf
from .photo_map import PhotoMap,photo_samples


PREFIX='pylon_rover_visual_'
OUT='/pylon/mun_rover/'


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def matrix(transform):
    t,q=transform.translation,transform.rotation
    result=np.eye(4)
    result[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    result[:3,3]=[t.x,t.y,t.z]
    return result


def visual_model(urdf):
    """Reuse the bridge's primitive-only validator and isolate all display TFs."""
    if len(urdf)>8*1024*1024 or '<!DOCTYPE' in urdf.upper() or '<!ENTITY' in urdf.upper():
        raise ValueError('invalid proxy URDF')
    tree=ET.fromstring(urdf)
    name=tree.attrib.get('name','')
    if not name.endswith('_active_vessel'):raise ValueError('expected active vessel proxy')
    root,links,joints=_parse_proxy_urdf(urdf,name[:-len('_active_vessel')])
    for element in tree.iter():
        for key in ('name','link'):
            if key in element.attrib:element.set(key,PREFIX+element.attrib[key])
    for visual in tree.findall('link/visual'):
        material=ET.SubElement(visual,'material',name='rover_silver')
        ET.SubElement(material,'color',rgba='0.65 0.76 0.86 1')
    return ET.tostring(tree,encoding='unicode'),root,joints,len(links)


def proxy_boxes(urdf,root,joints):
    """Enclose validated primitive visuals for conservative self-occlusion."""
    frames={root:np.eye(4)};pending=list(joints)
    while pending:
        for joint in pending[:]:
            if joint.parent_frame not in frames:continue
            pose=np.eye(4);pose[:3,3]=joint.translation
            pose[:3,:3]=Rotation.from_quat(joint.rotation).as_matrix()
            frames[joint.child_frame]=frames[joint.parent_frame]@pose;pending.remove(joint)
    boxes=[]
    for link in ET.fromstring(urdf).findall('link'):
        for visual in link.findall('visual'):
            origin=visual.find('origin');pose=np.eye(4)
            if origin is not None:
                pose[:3,3]=[float(v) for v in origin.get('xyz','0 0 0').split()]
                pose[:3,:3]=Rotation.from_euler('xyz',[float(v) for v in origin.get('rpy','0 0 0').split()]).as_matrix()
            shape=visual.find('geometry')[0]
            if shape.tag=='box':half=np.array([float(v) for v in shape.get('size').split()])/2
            elif shape.tag=='cylinder':half=np.array([float(shape.get('radius'))]*2+[float(shape.get('length'))/2])
            else:half=np.full(3,float(shape.get('radius')))
            boxes.append((frames[link.get('name')]@pose,half+.03))
    return boxes


class Visualization(Node):
    def __init__(self):
        super().__init__('pylon_mun_rover_visualization')
        self.declare_parameter('camera_sensor_id','auto')
        self.declare_parameter('photo_resolution',.1)
        self.declare_parameter('photo_max_range',12.)
        self.qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformSnapshot(self)
        self.description_pub=self.create_publisher(String,OUT+'robot_description',self.qos)
        self.photo_pub=self.create_publisher(PointCloud2,OUT+'photo_map',self.qos)
        self.image_pub=self.create_publisher(Image,OUT+'camera/image_raw',qos_profile_sensor_data)
        self.info_pub=self.create_publisher(CameraInfo,OUT+'camera/camera_info',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,OUT+'visualization_status',self.qos)
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='rover_photos')
        self.future=None;self.token=0;self.epoch=None;self.generation=None;self.geometry=None
        self.model_root='';self.model_links=0;self.last_model='';self.model_reason='waiting_for_model';self.model_boxes=[]
        self.camera_topic='';self.camera_subscriptions=[];self.reason='waiting_for_camera'
        self.frames=deque(maxlen=12);self.infos=deque(maxlen=20)
        self.ground=np.empty((0,3));self.returns=np.empty((0,3));self.ground_time=0.
        self.ready=False;self.status_time=0.;self.last_capture=None;self.last_attempt=0.
        resolution=float(self.get_parameter('photo_resolution').value)
        if not .05<=resolution<=.5:raise ValueError('photo_resolution must be 0.05..0.5 m')
        self.photos=PhotoMap(resolution=resolution)
        self.create_subscription(String,'/ksp_vessel/robot_description',self.model,self.qos)
        self.create_subscription(VesselLifecycle,'/ksp_vessel/lifecycle',self.lifecycle,self.qos)
        self.create_subscription(String,OUT+'status',self.navigation_status,self.qos)
        self.create_subscription(String,OUT+'geometry',self.geometry_message,self.qos)
        self.create_subscription(PointCloud2,OUT+'ground',self.ground_message,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,OUT+'points',self.returns_message,qos_profile_sensor_data)
        self.create_timer(.1,self.model_transform)
        self.create_timer(.2,self.photograph)
        self.create_timer(1.,self.discover_camera)
        self.create_timer(1.,self.report)
        self.publish_photos()

    def reset_photos(self):
        self.token+=1;self.photos=PhotoMap(resolution=self.photos.resolution)
        self.ground=np.empty((0,3));self.returns=np.empty((0,3));self.ground_time=0.
        self.frames.clear();self.infos.clear();self.last_capture=None;self.geometry=None
        self.publish_photos()

    def lifecycle(self,msg):
        epoch=(msg.vessel_id,msg.generation)
        if epoch!=self.epoch:
            self.reset_photos();self.clear_model();self.buffer.clear();self.epoch=epoch
        if not msg.model_ready:self.clear_model()

    def navigation_status(self,msg):
        data=json.loads(msg.data);generation=data.get('estimator_generation')
        if generation!=self.generation:
            self.reset_photos();self.generation=generation
        self.ready=bool(data.get('ready'));self.status_time=time.monotonic()

    def geometry_message(self,msg):
        data=json.loads(msg.data)
        if data.get('estimator_generation')==self.generation:self.geometry=data

    def clear_model(self):
        self.model_root='';self.model_links=0;self.last_model='';self.model_boxes=[]
        self.model_reason='waiting_for_model'
        # RViz's RobotModel must receive valid XML even while the vessel is
        # unavailable. A geometry-free link clears the display without parsing
        # an empty document during startup/lifecycle transitions.
        self.description_pub.publish(String(data='<robot name="rover_unavailable"><link name="pylon_rover_base_link"/></robot>'))
        self.static_tf.clear()

    def model(self,msg):
        if not msg.data:self.clear_model();return
        if msg.data==self.last_model:return
        try:description,root,joints,count=visual_model(msg.data)
        except (ValueError,ET.ParseError) as error:self.model_reason=str(error);return
        self.model_root=root;self.model_links=count;self.last_model=msg.data
        self.model_boxes=proxy_boxes(msg.data,root,joints)
        transforms=[]
        for joint in joints:
            tf=TransformStamped();tf.header.frame_id=PREFIX+joint.parent_frame
            tf.child_frame_id=PREFIX+joint.child_frame
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=joint.translation
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=joint.rotation
            transforms.append(tf)
        self.static_tf.clear()
        if transforms:self.static_tf.sendTransform(transforms)
        self.description_pub.publish(String(data=description))

    def model_transform(self):
        if not self.model_root:return
        try:
            # Only the relative physical mount is borrowed from the bridge.
            # No map -> base_link edge is added to its ground-truth TF tree.
            tf=self.buffer.lookup_transform('base_link',self.model_root,Time())
            estimated=self.buffer.lookup_transform('map','pylon_rover_base_link',Time())
            if abs(seconds(tf.header.stamp)-seconds(estimated.header.stamp))>1.:return
            tf.header.frame_id='pylon_rover_base_link';tf.child_frame_id=PREFIX+self.model_root
            tf.header.stamp=estimated.header.stamp;self.tf.sendTransform(tf)
            self.model_reason=''
        except TransformException:self.model_reason='waiting_for_model_transform'

    def discover_camera(self):
        selected=self.get_parameter('camera_sensor_id').value
        if selected not in ('','auto'):topic='/ksp_vessel/camera/'+selected+'/image_raw'
        else:
            topics=sorted(name for name,types in self.get_topic_names_and_types()
                if name.startswith('/ksp_vessel/camera/') and name.endswith('/image_raw')
                and 'sensor_msgs/msg/Image' in types and self.count_publishers(name)>0)
            topic=topics[0] if len(topics)==1 else ''
            if not topic:self.reason='multiple_cameras: set camera_sensor_id' if topics else 'waiting_for_camera'
        if topic==self.camera_topic:return
        for sub in self.camera_subscriptions:self.destroy_subscription(sub)
        self.camera_subscriptions=[];self.camera_topic=topic;self.frames.clear();self.infos.clear()
        self.last_capture=None
        if topic:
            self.camera_subscriptions=[
                self.create_subscription(Image,topic,self.image_message,QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)),
                self.create_subscription(CameraInfo,topic.rsplit('/',1)[0]+'/camera_info',self.info_message,qos_profile_sensor_data)]
            self.get_logger().info('Selected camera: '+topic)

    def image_message(self,msg):
        self.frames.append(msg);self.image_pub.publish(msg)

    def info_message(self,msg):
        self.infos.append(msg);self.info_pub.publish(msg)

    def ground_message(self,msg):
        if msg.header.frame_id!='map':return
        self.ground=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).copy()
        self.ground_time=time.monotonic()

    def returns_message(self,msg):
        if msg.header.frame_id=='map':
            self.returns=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).copy()

    def photograph(self):
        if self.future is not None:
            if not self.future.done():return
            try:
                samples=self.future.result()
                if self.job_token==self.token:
                    count=self.photos.add(*samples)
                    if count:self.last_capture=self.job_pose;self.publish_photos();self.reason=''
                    else:self.reason='no_new_visible_ground'
            except Exception as error:self.reason='photo_projection: '+str(error)
            self.future=None
        now=time.monotonic()
        if not self.ready or now-self.status_time>2.:self.reason='waiting_for_navigation_estimate';return
        if now-self.ground_time>2. or self.geometry is None:self.reason='waiting_for_observed_ground';return
        if not self.model_root:self.reason='waiting_for_vehicle_shape';return
        if now-self.last_attempt<.8 or not self.frames or not self.infos:return
        self.last_attempt=now
        for frame in reversed(self.frames):
            stamp=seconds(frame.header.stamp)
            info=min(self.infos,key=lambda m:abs(seconds(m.header.stamp)-stamp))
            if abs(seconds(info.header.stamp)-stamp)>.001 or info.header.frame_id!=frame.header.frame_id:continue
            try:
                at=Time.from_msg(frame.header.stamp)
                pose=self.buffer.lookup_transform('map','pylon_rover_base_link',at)
                mount=self.buffer.lookup_transform('base_link',frame.header.frame_id,at)
                root_mount=self.buffer.lookup_transform('base_link',self.model_root,at)
            except TransformException:continue
            body=matrix(pose.transform);camera=body@matrix(mount.transform)
            map_root=body@matrix(root_mount.transform)
            occluders=[(map_root@pose,half) for pose,half in self.model_boxes]
            if self.last_capture is not None:
                moved=np.linalg.norm(body[:3,3]-self.last_capture[:3,3])
                turned=Rotation.from_matrix(body[:3,:3]@self.last_capture[:3,:3].T).magnitude()
                if moved<.5 and turned<math.radians(8):return
            if frame.encoding!='rgb8' or frame.step<frame.width*3 or len(frame.data)!=frame.step*frame.height:
                self.reason='camera_requires_rgb8';return
            if info.width!=frame.width or info.height!=frame.height or np.any(np.asarray(info.d)!=0):
                self.reason='unsupported_camera_calibration';return
            rgb=np.frombuffer(bytes(frame.data),np.uint8).reshape(frame.height,frame.step)[:,:frame.width*3].reshape(frame.height,frame.width,3)
            self.job_token=self.token;self.job_pose=body
            self.future=self.worker.submit(photo_samples,self.ground,self.returns,camera,list(info.k),rgb,body,
                self.geometry['footprint'],self.geometry['rear_x'],self.photos.resolution,
                float(self.get_parameter('photo_max_range').value),occluders)
            return
        self.reason='waiting_for_camera_pose_at_image_time'

    def publish_photos(self):
        data=self.photos.cloud();msg=PointCloud2()
        msg.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='map')
        msg.height=1;msg.width=len(data);msg.point_step=16;msg.row_step=16*len(data)
        msg.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,name in enumerate(('x','y','z','rgb'))]
        msg.is_dense=True;msg.data=array('B',data.tobytes());self.photo_pub.publish(msg)

    def report(self):
        self.status_pub.publish(String(data=json.dumps(dict(camera_topic=self.camera_topic,
            model_links=self.model_links,model_reason=self.model_reason,photo_reason=self.reason,
            photo_frames=self.photos.frames,photo_cells=int(np.count_nonzero(self.photos.score)),
            photo_area_m2=float(np.count_nonzero(self.photos.score)*self.photos.resolution**2),
            estimator_generation=self.generation))))

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=Visualization()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
