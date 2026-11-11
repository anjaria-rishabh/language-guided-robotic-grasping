"""Camera-space localization to time-correct, base-frame target PoseStamped.

No motion commands are sent by this node. A bbox and RGB-D pixel do NOT yield
an object's 6-DOF grasp orientation. Pose orientation is identity placeholder.
"""
import json
import math
import traceback
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException

from .geometry import Intrinsics, deproject, transform_point
from .depth import depth_meters, patch_median_depth
from .grounders import ColorGrounder, ZeroShotGrounder


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('tops_target_localizer')
        defaults = {
            'rgb_topic':'/camera/image_raw', 'depth_topic':'/camera/depth',
            'camera_info_topic':'/camera/camera_info',
            'instruction_topic':'/tops/autonomy/instruction',
            'target_topic':'/tops/autonomy/target_pose',
            'base_frame':'panda_link0', 'grounder':'color',
            'model':'google/owlvit-base-patch32', 'min_score':0.2,
            'min_depth_m':0.10, 'max_depth_m':2.5,
            'sync_slop_s':0.07, 'camera_info_age_s':2.0,
            'tf_timeout_s':0.08, 'publish_period_s':0.1,
            'max_image_age_s':0.4, 'min_valid_pixels':12}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p = {k:self.get_parameter(k).value for k in defaults}
        self.bridge = CvBridge()
        self.camera_info = None
        self.last_camera_info_stamp = None
        self.instruction = ''
        self.last_pair = None
        self.last_published_stamp = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grounder = (ColorGrounder() if self.p['grounder']=='color'
                         else ZeroShotGrounder(self.p['model'], self.p['min_score']))
        self.publisher = self.create_publisher(PoseStamped, self.p['target_topic'], 10)
        self.debug_pub = self.create_publisher(String, '/tops/autonomy/detection', 10)
        self.create_subscription(String, self.p['instruction_topic'], self.on_instruction, 10)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                 self.on_camera_info, qos_profile_sensor_data)
        rgb = Subscriber(self, Image, self.p['rgb_topic'], qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, self.p['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb,depth],queue_size=12,
                                                slop=self.p['sync_slop_s'])
        self.sync.registerCallback(self.on_pair)
        self.create_timer(self.p['publish_period_s'], self.process)
        self.get_logger().info('Localization is passive; no robot motion commanded')

    def on_instruction(self, msg):
        self.instruction = msg.data.strip()
        self.last_published_stamp = None
        self.get_logger().info('Target query updated: '+self.instruction)

    def on_camera_info(self, msg):
        self.camera_info = msg
        self.last_camera_info_stamp = self.get_clock().now()

    def on_pair(self, rgb, depth):
        self.last_pair = (rgb,depth)

    def process(self):
        if not self.instruction or self.last_pair is None or self.camera_info is None:
            return
        rgb,depth = self.last_pair
        stamp = Time.from_msg(rgb.header.stamp)
        if stamp.nanoseconds == 0: return
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if age < -0.1 or age > self.p['max_image_age_s']: return
        if (self.p['camera_info_age_s'] > 0.0 and
            (self.get_clock().now()-self.last_camera_info_stamp).nanoseconds*1e-9 > self.p['camera_info_age_s']):
            return
        if abs((Time.from_msg(depth.header.stamp)-stamp).nanoseconds)*1e-9 > self.p['sync_slop_s']:
            return
        if self.last_published_stamp == stamp.nanoseconds:
            return  # Never republish an old observation as if fresh.
        if not rgb.header.frame_id or not depth.header.frame_id:
            return
        # Depth must be registered/aligned to RGB; do not use unrelated optical frames.
        if rgb.header.frame_id != depth.header.frame_id:
            self.get_logger().warn('RGB and depth frame IDs differ; use registered depth')
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            depth_array = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            if bgr.shape[:2] != depth_array.shape[:2]:
                return
            detection = self.grounder.detect(bgr, self.instruction)
            if detection is None or detection.score < self.p['min_score']:
                return
            z_pixel = patch_median_depth(depth_meters(depth_array, depth.encoding),
                         detection.bbox, min_valid_pixels=self.p['min_valid_pixels'],
                         near=self.p['min_depth_m'], far=self.p['max_depth_m'])
            if z_pixel is None: return
            z,u,v = z_pixel
            k = self.camera_info.k
            intrinsics = Intrinsics(k[0],k[4],k[2],k[5])
            point_cam = deproject(u,v,z,intrinsics)
            tf = self.tf_buffer.lookup_transform(
                self.p['base_frame'], rgb.header.frame_id, stamp,
                timeout=Duration(seconds=self.p['tf_timeout_s']))
            rotation = tf.transform.rotation
            translation = tf.transform.translation
            point_base = transform_point(point_cam,
                (rotation.x,rotation.y,rotation.z,rotation.w),
                (translation.x,translation.y,translation.z))
            if not all(math.isfinite(x) for x in point_base): return
            # CPU zero-shot inference can be slow; never publish an old detection.
            inference_age = (self.get_clock().now()-stamp).nanoseconds * 1e-9
            if inference_age < 0 or inference_age > self.p['max_image_age_s']:
                return
            pose = PoseStamped()
            pose.header.stamp = rgb.header.stamp
            pose.header.frame_id = self.p['base_frame']
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point_base
            pose.pose.orientation.w = 1.0  # NOT a measured object orientation.
            self.publisher.publish(pose)
            self.last_published_stamp = stamp.nanoseconds
            debug = {'instruction':self.instruction, 'label':detection.label,
                     'score':detection.score, 'bbox':list(detection.bbox),
                     'camera_depth_m':z, 'base_xyz_m':list(point_base),
                     'stamp_ns':stamp.nanoseconds}
            self.debug_pub.publish(String(data=json.dumps(debug)))
        except TransformException as ex:
            self.get_logger().warn('TF at measurement time unavailable: '+str(ex))
        except Exception as ex:
            self.get_logger().error('Perception failure: '+str(ex))


def main():
    rclpy.init()
    node = PerceptionNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
