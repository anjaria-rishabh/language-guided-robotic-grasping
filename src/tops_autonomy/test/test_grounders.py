import cv2
import numpy as np
from tops_autonomy.grounders import ColorGrounder,target_phrase


def test_simple_instruction():
    assert target_phrase('Please pick up the red cube.')=='red cube'


def test_color_demo_box():
    bgr=np.zeros((120,140,3),dtype=np.uint8)
    cv2.rectangle(bgr,(30,40),(80,85),(0,0,255),-1)
    d=ColorGrounder().detect(bgr,'Pick up the red cube')
    assert d is not None
    assert d.bbox==(30,40,81,86)
    assert ColorGrounder().detect(bgr,'green cube') is None


def test_no_language_color_returns_none():
    assert ColorGrounder().detect(np.zeros((30,30,3),dtype=np.uint8),'mug') is None
