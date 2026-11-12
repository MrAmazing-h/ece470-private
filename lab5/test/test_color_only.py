"""Color-only regressions; no camera, robot, calibration, or IO is required."""

import cv2
import numpy as np
import pytest
from ece470labs.color_vision import DEFAULT_HSV, annotate, detect_color


def settings():
    return dict(hsv=DEFAULT_HSV,roi=[40,30,600,440],area_px=[100,1200],morphology_kernel=3)


def hsv_color(h,s=210,v=220):
    return tuple(int(x) for x in cv2.cvtColor(np.uint8([[[h,s,v]]]),cv2.COLOR_HSV2BGR)[0,0])


def test_two_round_calibration_targets_are_not_rejected_by_circularity():
    image=np.zeros((480,640,3),np.uint8)
    for center in [(190,170),(290,170)]:
        cv2.circle(image,center,8,hsv_color(8),-1)
    points,mask=detect_color(image,'orange',settings())
    assert len(points)==2
    np.testing.assert_allclose([p.pt for p in points],[[190,170],[290,170]],atol=.2)


def test_color_argument_selects_distinct_masks_and_rejects_large_triangle():
    image=np.zeros((480,640,3),np.uint8)
    for x,hue in [(100,58),(200,28),(300,110),(400,179)]:
        cv2.rectangle(image,(x-10,100),(x+10,120),hsv_color(hue),-1)
    cv2.fillPoly(image,[np.array([[450,180],[560,180],[560,320]])],hsv_color(28))
    for color,x in [('green',100),('yellow',200),('blue',300),('red',400)]:
        points,mask=detect_color(image,color,settings())
        assert len(points)==1
        np.testing.assert_allclose(points[0].pt,[x,110],atol=.2)
    assert len(detect_color(image,'orange',settings())[0])==0


def test_roi_excludes_orange_objects_outside_the_table():
    image=np.zeros((480,640,3),np.uint8)
    for center in [(20,20),(200,200),(620,450)]:
        cv2.circle(image,center,8,hsv_color(8),-1)
    points,_=detect_color(image,'orange',settings())
    assert len(points)==1
    np.testing.assert_allclose(points[0].pt,[200,200],atol=.2)


def test_both_red_hue_wrap_ranges_are_detected():
    image=np.zeros((480,640,3),np.uint8)
    for x,hue in [(100,1),(200,179)]:
        cv2.rectangle(image,(x-10,100),(x+10,120),hsv_color(hue),-1)
    assert len(detect_color(image,'red',settings())[0])==2


def test_invalid_color_and_image_are_rejected():
    image=np.zeros((480,640,3),np.uint8)
    with pytest.raises(ValueError):
        detect_color(image,'unknown',settings())
    with pytest.raises(ValueError):
        detect_color(None,'green',settings())


def test_overlay_keeps_input_unchanged_and_reports_counts():
    image=np.zeros((480,640,3),np.uint8)
    cv2.rectangle(image,(100,100),(120,120),hsv_color(58),-1)
    before=image.copy()
    overlay,masks,centers=annotate(image,['green','orange'],settings())
    assert len(centers['green'])==1 and centers['orange']==[]
    np.testing.assert_array_equal(image,before)
    assert not np.array_equal(image,overlay)
