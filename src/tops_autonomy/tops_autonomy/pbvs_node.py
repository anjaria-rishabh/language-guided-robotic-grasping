"""Passive PBVS reference generator; outputs TwistStamped, NOT robot actuators."""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool
from .geometry import pbvs_command


class PBVSNode(Node):
    def __init__(self):
        super().__init__('tops_pbvs_reference')
        defaults = {
            'target_topic':'/tops/autonomy/target_pose',
            'ee_topic':'/tops/autonomy/ee_pose',
            'mode_topic':'/tops/autonomy/mode',
            'twist_topic':'/tops/autonomy/pbvs_twist',
            'base_frame':'panda_link0',
            'enabled':False, 'require_servo_mode':True,
            'update_rate_hz':30.0, 'max_age_s':0.3,
            'pregrasp_offset_z_m':0.12,
            'desired_orientation_xyzw':[1.0,0.0,0.0,0.0],
            'kp_linear':0.8, 'kp_angular':0.8,
            'max_linear_mps':0.07, 'max_angular_radps':0.25,
            'position_tolerance_m':0.003, 'orientation_tolerance_rad':0.05,
            'workspace_min_xyz':[0.18,-0.50,0.01],
            'workspace_max_xyz':[0.85,0.50,0.75],
        }
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p = {k:self.get_parameter(k).value for k in defaults}
        self.target = None
        self.ee = None
        self.mode = ''
        self.pub = self.create_publisher(TwistStamped,self.p['twist_topic'],10)
        self.done = self.create_publisher(Bool,'/tops/autonomy/aligned',10)
        self.create_subscription(PoseStamped,self.p['target_topic'],self.on_target,10)
        self.create_subscription(PoseStamped,self.p['ee_topic'],self.on_ee,10)
        self.create_subscription(String,self.p['mode_topic'],self.on_mode,10)
        self.create_timer(1.0/self.p['update_rate_hz'],self.tick)
        self.get_logger().warn('PBVS outputs references only: actuator adapter NOT active')

    def on_target(self,msg): self.target = msg
    def on_ee(self,msg): self.ee = msg
    def on_mode(self,msg): self.mode = msg.data.strip().upper()

    def fresh(self,msg):
        if msg is None or msg.header.frame_id != self.p['base_frame']:
            return False
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0: return False
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        return 0 <= age <= self.p['max_age_s']

    def tick(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.p['base_frame']
        aligned = False
        ready = self.p['enabled'] and self.fresh(self.target) and self.fresh(self.ee)
        ready = ready and (not self.p['require_servo_mode'] or self.mode == 'SERVO')
        if ready:
            obj = self.target.pose.position
            ee = self.ee.pose.position
            goal = (obj.x, obj.y, obj.z+self.p['pregrasp_offset_z_m'])
            minimum = self.p['workspace_min_xyz']; maximum=self.p['workspace_max_xyz']
            ready = all(a <= p <= b for p,a,b in zip(goal,minimum,maximum))
            if ready:
                q = self.ee.pose.orientation
                try:
                    lin,ang,aligned = pbvs_command(
                        goal,(ee.x,ee.y,ee.z), self.p['desired_orientation_xyzw'],
                        (q.x,q.y,q.z,q.w),
                        self.p['kp_linear'],self.p['kp_angular'],
                        self.p['max_linear_mps'],self.p['max_angular_radps'],
                        self.p['position_tolerance_m'],self.p['orientation_tolerance_rad'])
                    (twist.twist.linear.x,twist.twist.linear.y,twist.twist.linear.z)=lin
                    (twist.twist.angular.x,twist.twist.angular.y,twist.twist.angular.z)=ang
                except ValueError:
                    pass
        # Explicit zeros on any stale, unsafe, inactive or completed condition.
        self.pub.publish(twist)
        self.done.publish(Bool(data=aligned and ready))


def main():
    rclpy.init()
    node=PBVSNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
