"""Opt-in, schema-checked tops_msgs/GraspTarget bridge.

The exact upstream message definition is not bundled in the proposal PDFs.
Rather than guess fields and silently publish an invalid contract, we print
its discovered schema and require an explicitly configured PoseStamped field.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ContractBridge(Node):
    def __init__(self):
        super().__init__('tops_grasp_target_bridge')
        from tops_msgs.msg import GraspTarget
        self.msg_type = GraspTarget
        self.declare_parameter('input_topic','/tops/autonomy/target_pose')
        self.declare_parameter('output_topic','/tops/grasp_target')
        self.declare_parameter('pose_field','')
        self.declare_parameter('label_field','')
        self.declare_parameter('confidence_field','')
        self.declare_parameter('enabled',False)
        fields = self.msg_type.get_fields_and_field_types()
        self.get_logger().info('Installed tops_msgs/GraspTarget fields: '+str(fields))
        self.fields=fields
        self.enabled = bool(self.get_parameter('enabled').value)
        self.pose_field = str(self.get_parameter('pose_field').value)
        self.label_field = str(self.get_parameter('label_field').value)
        self.confidence_field = str(self.get_parameter('confidence_field').value)
        if self.enabled:
            if self.fields.get(self.pose_field) not in ('geometry_msgs/PoseStamped',
                                                        'geometry_msgs/msg/PoseStamped'):
                raise RuntimeError('Configured pose_field must be a PoseStamped field '
                                   'in installed GraspTarget; contract='+str(fields))
            if self.label_field and self.fields.get(self.label_field) != 'string':
                raise RuntimeError('label_field is not a string field in GraspTarget')
            if self.confidence_field and self.fields.get(self.confidence_field) not in ('float32','float64'):
                raise RuntimeError('confidence_field is not floating point')
            self.pub=self.create_publisher(GraspTarget,
                                   self.get_parameter('output_topic').value, 10)
            self.create_subscription(PoseStamped,
                 self.get_parameter('input_topic').value,self.on_pose,10)
        else:
            self.get_logger().warn('Bridge disabled; inspect schema and explicitly map fields first')

    def on_pose(self,pose):
        out=self.msg_type()
        setattr(out,self.pose_field,pose)
        # Do not invent label/confidence fields without subscribed validated metadata.
        self.pub.publish(out)


def main():
    rclpy.init()
    n=ContractBridge()
    try: rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()
