"""Pure image-math helpers; never turn missing depth into a valid target."""
import numpy as np


def depth_meters(raw, encoding):
    if encoding in ('16UC1', 'mono16'):
        return np.asarray(raw, dtype=np.float32) * 0.001
    if encoding == '32FC1':
        return np.asarray(raw, dtype=np.float32)
    raise ValueError('Unsupported depth encoding: ' + str(encoding))


def patch_median_depth(depth_m, bbox, center_fraction=0.45,
                       min_valid_pixels=12, near=0.1, far=3.0):
    """Median of central bbox ROI, filtering NaN/zero/out-of-range pixels.

    A central ROI reduces background edge contamination. Returns (z, u, v)
    or None for insufficient data. Project the same (u,v) as the sample ROI.
    """
    image = np.asarray(depth_m)
    if image.ndim != 2:
        raise ValueError('Expected 2-D aligned depth image')
    height, width = image.shape
    x1,y1,x2,y2 = bbox
    if x2 <= x1 or y2 <= y1: return None
    cx, cy = (x1+x2)/2, (y1+y2)/2
    hw = max(1, int((x2-x1)*center_fraction/2))
    hh = max(1, int((y2-y1)*center_fraction/2))
    xl, xh = max(0, int(cx)-hw), min(width, int(cx)+hw+1)
    yl, yh = max(0, int(cy)-hh), min(height, int(cy)+hh+1)
    if xh <= xl or yh <= yl: return None
    roi = image[yl:yh, xl:xh]
    ok = roi[np.isfinite(roi) & (roi >= near) & (roi <= far)]
    if ok.size < min_valid_pixels: return None
    return float(np.median(ok)), float(cx), float(cy)
