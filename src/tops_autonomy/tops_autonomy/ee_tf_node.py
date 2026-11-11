"""Bridge TF-based Panda end-effector pose into the PBVS input interface.

Source is the live robot TF tree. The sampled transform's original timestamp
is retained; it is not artificially marked fresh if joint-state TF is stale.
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class EndEffectorTFNode(Node):
    def __init__(self):
        super().__init__('tops_ee_tf_bridge')
        self.declare_parameter('base_frame','panda_link0')
        self.declare_parameter('ee_frame','panda_hand')
        self.declare_parameter('output_topic','/tops/autonomy/ee_pose')
        self.declare_parameter('publish_rate_hz',30.0)
        self.declare_parameter('max_tf_age_s',0.25)
        self.base=self.get_parameter('base_frame').value
        self.ee=self.get_parameter('ee_frame').value
        hz=float(self.get_parameter('publish_rate_hz').value)
        self.max_age=float(self.get_parameter('max_tf_age_s').value)
        self.pub=self.create_publisher(PoseStamped,
                 self.get_parameter('output_topic').value,10)
        self.buffer=Buffer(cache_time=Duration(seconds=5.0))
        self.listener=TransformListener(self.buffer,self)
        self.create_timer(1.0/hz,self.tick)
        self.last_warning_ns=0

    def tick(self):
        try:
            tf=self.buffer.lookup_transform(self.base,self.ee,Time(),
                                            timeout=Duration(seconds=0.03))
            stamp=Time.from_msg(tf.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds*1e-9
            if stamp.nanoseconds==0 or age < 0 or age > self.max_age:
                return
            pose=PoseStamped()
            pose.header=tf.header
            pose.pose.position.x=tf.transform.translation.x
            pose.pose.position.y=tf.transform.translation.y
            pose.pose.position.z=tf.transform.translation.z
            pose.pose.orientation=tf.transform.rotation
            self.pub.publish(pose)
        except TransformException:
            pass  # Failure to locate live TF deliberately means no fresh EE pose.


def main():
    rclpy.init()
    node=EndEffectorTFNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
