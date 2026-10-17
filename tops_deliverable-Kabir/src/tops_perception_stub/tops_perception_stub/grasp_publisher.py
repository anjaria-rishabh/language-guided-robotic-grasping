#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
A stand-in for the perception and planning half of the system.

---------------------------------------------------------------------------
What this is, and what it is not
---------------------------------------------------------------------------
It is NOT an attempt to do Rishabh's job badly. There is no vision here at
all: the object poses are read straight out of the simulator, which is
exactly the information a perfect perception system would produce.

What it adds on top is the thing that makes the experiment an experiment:
CONTROLLED, REPRODUCIBLE ERROR. Four disturbance regimes, each with a known
magnitude, applied to a ground-truth pose with a seeded random number
generator.

That inversion is the whole point. A real vision pipeline has error you
cannot measure and cannot repeat, which means you can never say "the
compliant controller tolerated 8 mm of calibration drift" -- you can only
say it did better on average, without knowing what it was tolerating.
Injecting the error deliberately turns the disturbance into an independent
variable.

When Rishabh's pipeline is ready it publishes the same GraspTarget message
on the same topic, this node is not launched, and nothing downstream
changes. That is the point of the interface living in tops_msgs.

---------------------------------------------------------------------------
The four regimes
---------------------------------------------------------------------------
  clean          ground truth. The control condition. Establishes that the
                 stack works at all, and sets the ceiling the others are
                 measured against.

  sensor_noise   zero-mean Gaussian jitter, resampled every frame. Models
                 depth-sensing noise. The target dances; a stiff controller
                 chases the dance and a compliant one filters it through
                 its own dynamics.

  calibration    a CONSTANT offset, fixed for the whole trial, resampled
                 per trial. Models a miscalibrated camera-to-base
                 transform. This is the nastiest regime and the one the
                 project's claim is really about: the error does not
                 average out, so the arm confidently drives to the wrong
                 place and touches the object off-centre every time.

  displacement   the object is moved after the target is first published.
                 Models something being nudged mid-approach.

    ros2 run tops_perception_stub grasp_publisher --ros-args \
        -p regime:=calibration -p seed:=17
"""
import math
import random

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from visualization_msgs.msg import Marker

from tops_msgs.msg import GraspTarget, SupervisorState

REGIMES = ('clean', 'sensor_noise', 'calibration', 'displacement')


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """ZYX intrinsic, the ROS convention. Written out rather than pulled
    from tf_transformations, which is not in the apt ROS 2 distribution and
    would make this package need a pip install to run."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class GraspPublisher(Node):

    def __init__(self):
        super().__init__('grasp_publisher')

        self.declare_parameter('regime', 'clean')
        self.declare_parameter('seed', 0)
        # 60 Hz, not the camera's nominal 30. The extra headroom costs
        # nothing and keeps the gap between targets comfortably inside the
        # supervisor's watchdog even when Gazebo's /clock advances coarsely.
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('object_label', 'red block')
        # Ground-truth object position in the base frame. The trial runner
        # sets this per trial from the scene it spawned.
        # In the ARM BASE frame (panda_link0), not the world frame. The arm
        # base sits at world (-0.05, 0, 0.75), so a block resting at world
        # (0.45, -0.15, 0.776) is at (0.50, -0.15, 0.026) here. Publishing
        # world coordinates into a base-frame topic put every target 50 mm
        # off in x, which is a silent, systematic bias on every trial.
        self.declare_parameter('object_xyz', [0.50, -0.15, 0.026])
        # Where the TCP should end up relative to the object centre. The
        # gripper approaches from above, so this is a positive z offset.
        # Slightly BELOW the object's centre.
        #
        # This was +15 mm, which is wrong for a top-down grasp and produced
        # a hand that collided with the object instead of straddling it. The
        # TCP sits between the fingertips near their far end, so commanding
        # the object centre places the fingers either side of the body. A
        # positive offset raises the TCP toward the top face, and for a
        # 50 mm cube whose top is 25 mm above its centre that leaves the
        # fingers closing on the upper corner rather than the sides.
        # The fingertips reach only 8 mm beyond the TCP, so commanding the
        # block's centre leaves them pinching its top 20 mm — a shallow grip
        # on the upper corner, which pushes the block away instead of
        # holding it. Going 10 mm lower puts the fingers around most of the
        # body while still clearing the tabletop.
        self.declare_parameter('grasp_offset_z', -0.010)
        self.declare_parameter('approach_height', 0.12)
        # How fast the commanded target descends, m/s, once the arm is above
        # the object. See _tick() for why this is a ramp and not a step.
        self.declare_parameter('descent_speed', 0.05)

        self.declare_parameter('noise_stddev_m', 0.004)
        self.declare_parameter('calibration_offset_m', 0.008)
        self.declare_parameter('displacement_m', 0.015)
        self.declare_parameter('displacement_after_s', 2.0)
        self.declare_parameter('confidence', 0.95)

        self.regime = self.get_parameter('regime').value
        if self.regime not in REGIMES:
            raise ValueError(
                f"regime '{self.regime}' is not one of {REGIMES}")

        self.seed = int(self.get_parameter('seed').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.object_label = self.get_parameter('object_label').value
        self.object_xyz = list(self.get_parameter('object_xyz').value)
        self.grasp_offset_z = float(self.get_parameter('grasp_offset_z').value)
        self.approach_height = float(self.get_parameter('approach_height').value)
        self.descent_speed = float(self.get_parameter('descent_speed').value)
        self.noise_stddev = float(self.get_parameter('noise_stddev_m').value)
        self.calibration_offset = float(self.get_parameter('calibration_offset_m').value)
        self.displacement = float(self.get_parameter('displacement_m').value)
        self.displacement_after = float(self.get_parameter('displacement_after_s').value)
        self.confidence = float(self.get_parameter('confidence').value)

        # A seeded generator per trial. This is what makes the whole
        # experiment reproducible, and it is also what makes PAIRED
        # statistics possible: run trial 17 with seed 17 under both
        # controllers and they see the identical disturbance, which is a
        # substantially more powerful comparison for the same number of
        # runs (Theory 10 §10.2).
        self.rng = random.Random(self.seed)

        # Drawn ONCE per trial, not per frame. That is what makes it a
        # calibration error rather than noise, and it is the difference
        # that matters: noise averages out over an approach, a fixed offset
        # does not.
        self.calibration_bias = self._random_unit_offset(self.calibration_offset)
        self.displacement_vector = self._random_unit_offset(self.displacement)

        self.publisher = self.create_publisher(GraspTarget, '/tops/grasp_target', 10)
        self.marker_pub = self.create_publisher(Marker, '/tops/target_marker', 10)
        self.create_subscription(
            SupervisorState, '/tops/supervisor_state', self._on_supervisor, 10)

        self.start_time = self.get_clock().now()
        self.supervisor_state = SupervisorState.STATE_IDLE
        self.supervisor_time_in_state = 0.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"regime '{self.regime}', seed {self.seed}, object at "
            f'{self.object_xyz}. '
            + self._describe_disturbance())

    def _describe_disturbance(self) -> str:
        if self.regime == 'clean':
            return 'no disturbance: ground truth.'
        if self.regime == 'sensor_noise':
            return f'per-frame Gaussian noise, sigma {self.noise_stddev * 1000:.1f} mm.'
        if self.regime == 'calibration':
            b = self.calibration_bias
            return ('fixed bias for this trial: '
                    f'[{b[0] * 1000:+.1f}, {b[1] * 1000:+.1f}, {b[2] * 1000:+.1f}] mm.')
        d = self.displacement_vector
        return (f'object moves by [{d[0] * 1000:+.1f}, {d[1] * 1000:+.1f}, '
                f'{d[2] * 1000:+.1f}] mm after {self.displacement_after:.1f} s.')

    def _random_unit_offset(self, magnitude: float):
        """A random direction with a FIXED magnitude.

        Fixed rather than Gaussian on purpose: the experiment's independent
        variable is "how much error", so every trial in a regime must get
        the same amount of it, differing only in direction. A Gaussian
        magnitude would confound the size of the disturbance with the
        trial-to-trial variation the statistics are trying to measure.

        Mostly horizontal, because a vertical calibration error is caught
        immediately by the table and is not the interesting case.
        """
        angle = self.rng.uniform(0.0, 2.0 * math.pi)
        return [
            magnitude * math.cos(angle),
            magnitude * math.sin(angle),
            magnitude * 0.2 * self.rng.uniform(-1.0, 1.0),
        ]

    def _on_supervisor(self, msg: SupervisorState):
        self.supervisor_state = msg.state
        self.supervisor_time_in_state = msg.time_in_state

    def _perceived_object_position(self):
        """Ground truth plus whatever this regime's error is."""
        x, y, z = self.object_xyz
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if self.regime == 'sensor_noise':
            # Resampled every frame: this is noise, and it averages out.
            x += self.rng.gauss(0.0, self.noise_stddev)
            y += self.rng.gauss(0.0, self.noise_stddev)
            z += self.rng.gauss(0.0, self.noise_stddev * 0.5)

        elif self.regime == 'calibration':
            # Constant for the trial: this is bias, and it does NOT average
            # out. The arm goes confidently to the wrong place.
            x += self.calibration_bias[0]
            y += self.calibration_bias[1]
            z += self.calibration_bias[2]

        elif self.regime == 'displacement' and elapsed > self.displacement_after:
            x += self.displacement_vector[0]
            y += self.displacement_vector[1]
            z += self.displacement_vector[2]

        return x, y, z

    def _tick(self):
        x, y, z = self._perceived_object_position()

        msg = GraspTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.object_label = self.object_label
        msg.confidence = self.confidence

        # Two-stage approach. Until the supervisor is in the compliant
        # phase, aim at a standoff point above the object; then descend.
        #
        # This is not a refinement, it is a requirement: a straight-line
        # move from the home pose to the grasp point passes through the
        # tabletop. A real planner (MoveIt 2, on Rishabh's side) produces
        # the same shape of trajectory for the same reason.
        descending = self.supervisor_state in (
            SupervisorState.STATE_COMPLIANT,
            SupervisorState.STATE_GRASP,
            SupervisorState.STATE_LIFT,
            SupervisorState.STATE_HOLD,
        )
        # Two heights only: stand off above the object, then the grasp pose.
        #
        # There is deliberately no ramp here any more. Smoothing the motion
        # is the supervisor's job -- it slews the commanded equilibrium at a
        # bounded speed, which bounds the spring force at the same time.
        # Doing it in both places meant two interacting ramps, one of them
        # driven by this node's view of the supervisor's clock, which is
        # exactly the sort of coupling that is impossible to tune.
        #
        # Closing in laterally at a safe height first is NOT optional: a
        # straight line from the home pose to the grasp pose passes through
        # the tabletop.
        target_z = (z + self.grasp_offset_z) if descending else (z + self.approach_height)

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = target_z
        # Gripper pointing straight down: 180 degrees of roll from the base
        # frame's +z. The jaw is aligned along the object's x axis, which
        # for a cube is arbitrary and for anything else would come from the
        # grasp pose estimator.
        msg.pose.orientation = quaternion_from_euler(math.pi, 0.0, 0.0)

        # Always False. This node publishes targets and never observes the
        # arm, so it has no basis for claiming the servo loop has converged.
        # The supervisor decides from the error it actually measures.
        msg.final = False

        self.publisher.publish(msg)
        self._publish_marker(msg)

    def _publish_marker(self, target: GraspTarget):
        marker = Marker()
        marker.header = target.header
        marker.ns = 'grasp_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
