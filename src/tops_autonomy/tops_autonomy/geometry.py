"""ROS-independent geometry for metric deprojection and safe PBVS commands."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.fx, self.fy, self.cx, self.cy)):
            raise ValueError('Nonfinite camera intrinsics')
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError('Camera focal lengths must be positive')


def deproject(u, v, depth_m, camera):
    """Camera optical frame: +X right, +Y down, +Z forward (meters)."""
    if not math.isfinite(depth_m) or depth_m <= 0:
        raise ValueError('Depth must be positive and finite')
    return ((u-camera.cx)*depth_m/camera.fx,
            (v-camera.cy)*depth_m/camera.fy, depth_m)


def quaternion_normalize(q):
    n = math.sqrt(sum(x*x for x in q))
    if n < 1e-10 or not math.isfinite(n):
        raise ValueError('Invalid quaternion')
    return tuple(x/n for x in q)


def quaternion_multiply(a, b):
    x1,y1,z1,w1 = a
    x2,y2,z2,w2 = b
    return (w1*x2 + x1*w2 + y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2,
            w1*w2-x1*x2-y1*y2-z1*z2)


def quaternion_inverse(q):
    x,y,z,w = quaternion_normalize(q)
    return (-x,-y,-z,w)


def rotate(q, vector):
    q = quaternion_normalize(q)
    v = quaternion_multiply(quaternion_multiply(q, (*vector, 0.0)), quaternion_inverse(q))
    return v[:3]


def transform_point(point, rotation_xyzw, translation):
    r = rotate(rotation_xyzw, point)
    return tuple(a+b for a,b in zip(r, translation))


def axis_angle_error(q_des, q_now):
    """Shortest rotation vector in the common reference frame (rad)."""
    err = quaternion_normalize(quaternion_multiply(
        quaternion_normalize(q_des), quaternion_inverse(q_now)))
    if err[3] < 0: err = tuple(-x for x in err)
    xyz = err[:3]
    n = math.sqrt(sum(x*x for x in xyz))
    if n < 1e-10: return (0.0, 0.0, 0.0)
    angle = 2.0*math.atan2(n, err[3])
    return tuple(x*angle/n for x in xyz)


def saturate_norm(v, limit):
    if limit <= 0: raise ValueError('Limit must be positive')
    norm = math.sqrt(sum(x*x for x in v))
    return tuple(x * min(1.0, limit/norm) for x in v) if norm > 0 else tuple(v)


def pbvs_command(target_xyz, ee_xyz, desired_q, ee_q, kp_linear=0.8,
                 kp_angular=0.8, vmax=0.08, wmax=0.3,
                 position_tolerance=0.003, angle_tolerance=0.05):
    """Return linear velocity, angular velocity, done flag in BASE frame.

    Orientation is an explicitly configured desired end-effector orientation;
    a 3-D target centroid alone does not determine grasp orientation.
    """
    error = tuple(t-e for t,e in zip(target_xyz, ee_xyz))
    rotational_error = axis_angle_error(desired_q, ee_q)
    done = (math.dist(target_xyz, ee_xyz) <= position_tolerance
            and math.sqrt(sum(x*x for x in rotational_error)) <= angle_tolerance)
    if done:
        return (0.0,)*3, (0.0,)*3, True
    linear = saturate_norm(tuple(kp_linear*x for x in error), vmax)
    angular = saturate_norm(tuple(kp_angular*x for x in rotational_error), wmax)
    return linear, angular, False
