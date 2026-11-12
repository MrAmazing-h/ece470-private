"""Color-only image processing. No ROS robot commands or calibration calls."""

import json
from pathlib import Path
import os

import cv2
import numpy as np


DEFAULT_HSV = {
    'orange': [[4, 130, 130], [18, 255, 255]],
    'green': [[45, 100, 80], [80, 255, 255]],
    'yellow': [[20, 130, 130], [37, 255, 255]],
    'blue': [[95, 90, 45], [125, 255, 255]],
    'red': [[170, 140, 90], [179, 255, 255]],
}


def load_vision(path=None):
    if path is None:
        path = os.environ.get('LAB5_CONFIG')
    if path is None:
        candidate = Path(__file__).resolve().parents[5]/'lab5/config.json'
        if candidate.exists():
            path = candidate
    vision = dict(hsv={key:[list(x) for x in value] for key,value in DEFAULT_HSV.items()},
                  roi=None, area_px=[100, 1200], morphology_kernel=3,
                  flip_horizontal=False)
    if path:
        supplied = json.loads(Path(path).read_text()).get('vision', {})
        for key,value in supplied.items():
            if key == 'hsv':
                vision['hsv'].update(value)
            else:
                vision[key] = value
    return vision


def detect_color(image_bgr, color, vision=None):
    """Use a white mask and shape/area filters appropriate to each target.

    Orange calibration markers are round. Block masks use a wider shape
    tolerance, but reject large objects such as the yellow set square.
    Returns OpenCV keypoints in (column,row) order and the HSV mask.
    """
    vision = vision or load_vision()
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('Expected a BGR image.')
    if color not in vision['hsv']:
        raise ValueError('Unknown color: '+str(color))
    low,high = vision['hsv'][color]
    if len(low)!=3 or len(high)!=3 or any(not 0<=a<=b<=limit for a,b,limit in zip(low,high,[179,255,255])):
        raise ValueError('Invalid HSV bounds for '+color)
    hsv = cv2.cvtColor(image_bgr,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,np.array(low,dtype=np.uint8),np.array(high,dtype=np.uint8))
    if color == 'red':
        # Red crosses the OpenCV Hue wrap at 179 -> 0.
        mask |= cv2.inRange(hsv,np.array([0,low[1],low[2]],np.uint8),
                           np.array([3,high[1],high[2]],np.uint8))
    roi = vision.get('roi')
    if roi is not None:
        x0,y0,x1,y1 = map(int,roi)
        if not 0<=x0<x1<=image_bgr.shape[1] or not 0<=y0<y1<=image_bgr.shape[0]:
            raise ValueError('ROI must be inside the full image.')
        selected = np.zeros_like(mask)
        selected[y0:y1,x0:x1] = mask[y0:y1,x0:x1]
        mask = selected
    size = int(vision.get('morphology_kernel',3))
    if size>1:
        kernel = np.ones((size,size),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor,params.blobColor = True,255
    params.filterByArea = True
    minimum,maximum = vision.get('area_px',[100,1200])
    params.minArea = float(max(minimum, 90 if color=='orange' else 140))
    params.maxArea = float(min(maximum, 500 if color=='orange' else 1600))
    if params.minArea>=params.maxArea:
        raise ValueError('Invalid blob-area bounds.')
    params.filterByCircularity = True
    params.minCircularity = .60 if color=='orange' else .50
    params.maxCircularity = 1.0
    params.filterByConvexity,params.minConvexity = True,.85
    params.filterByInertia,params.minInertiaRatio = True,.45
    params.minThreshold,params.maxThreshold,params.thresholdStep = 0,256,20
    params.minDistBetweenBlobs = 8
    points = cv2.SimpleBlobDetector_create(params).detect(mask)
    return sorted(points,key=lambda point:(point.pt[0],point.pt[1])),mask


def annotate(image_bgr,colors,vision):
    image = cv2.flip(image_bgr,1) if vision.get('flip_horizontal',False) else image_bgr.copy()
    overlay = image.copy()
    masks,centers = {},{}
    palette = dict(orange=(0,100,255),green=(0,220,0),yellow=(0,220,220),
                   blue=(255,140,20),red=(0,0,255))
    for color in colors:
        points,masks[color] = detect_color(image,color,vision)
        centers[color] = [[float(p.pt[0]),float(p.pt[1])] for p in points]
        overlay = cv2.drawKeypoints(overlay,points,None,palette[color],
                                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        for index,point in enumerate(points,1):
            col,row=point.pt
            cv2.putText(overlay,f'{color[0].upper()}{index}',
                        (int(col)-12,max(12,int(row)-14)),cv2.FONT_HERSHEY_SIMPLEX,
                        .38,palette[color],1,cv2.LINE_AA)
    roi = vision.get('roi')
    if roi:
        cv2.rectangle(overlay,tuple(roi[:2]),tuple(roi[2:]),(220,220,220),1)
    summary='  '.join(f'{name}: {len(centers[name])}' for name in colors)
    cv2.putText(overlay,summary,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),3,cv2.LINE_AA)
    cv2.putText(overlay,summary,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
    return overlay,masks,centers
