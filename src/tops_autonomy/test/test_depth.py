import numpy as np
import pytest
from tops_autonomy.depth import depth_meters,patch_median_depth


def test_depth_units():
    assert depth_meters(np.array([[1000]],dtype='uint16'),'16UC1')[0,0]==pytest.approx(1.)
    assert depth_meters(np.array([[.5]],dtype='float32'),'32FC1')[0,0]==pytest.approx(.5)
    with pytest.raises(ValueError): depth_meters([[123]],'8UC1')


def test_patch_median_excludes_outliers():
    depth=np.ones((100,100),dtype=float)
    depth[49,49]=np.nan
    depth[50,50]=10
    result=patch_median_depth(depth,(30,30,70,70))
    assert result[0]==pytest.approx(1.0)
    assert result[1:]==pytest.approx((50,50))


def test_too_few_pixels_fail_closed():
    assert patch_median_depth(np.zeros((10,10)),(2,2,8,8)) is None
