import math
import pytest
from tops_autonomy.geometry import (Intrinsics,deproject,transform_point,
    quaternion_multiply,axis_angle_error,pbvs_command)


def test_camera_deprojection():
    intr=Intrinsics(500,500,320,240)
    assert deproject(320,240,1.2,intr)==pytest.approx((0,0,1.2))
    assert deproject(420,240,1.0,intr)==pytest.approx((0.2,0,1.0))


def test_depth_validation():
    with pytest.raises(ValueError): deproject(1,2,float('nan'),Intrinsics(1,1,0,0))
    with pytest.raises(ValueError): Intrinsics(0,1,0,0)


def test_transform_rotation_translation():
    q=(0,0,math.sin(math.pi/4),math.cos(math.pi/4))
    assert transform_point((1,0,0),q,(1,2,3))==pytest.approx((1,3,3))


def test_pbvs_gating_and_saturation():
    lin,ang,done=pbvs_command((1,0,0),(0,0,0),(0,0,0,1),(0,0,0,1),vmax=.1)
    assert not done
    assert math.isclose(math.dist(lin,(0,0,0)),.1)
    assert ang==pytest.approx((0,0,0))
    lin,ang,done=pbvs_command((0,0,0),(0,0,0),(0,0,0,1),(0,0,0,1))
    assert done and lin==(0.,0.,0.)


def test_shortest_rotation():
    assert axis_angle_error((0,0,0,-1),(0,0,0,1))==pytest.approx((0,0,0))
    q=(0,0,math.sin(math.pi/4),math.cos(math.pi/4))
    assert axis_angle_error(q,(0,0,0,1))==pytest.approx((0,0,math.pi/2))
