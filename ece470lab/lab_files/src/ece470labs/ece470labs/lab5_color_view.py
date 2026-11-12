#!/usr/bin/env python3
"""Image-only color preview: subscribes to Image and cannot command the robot."""

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from .color_vision import annotate, load_vision


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=os.environ.get('LAB5_CONFIG'))
    parser.add_argument('--colors',nargs='+',default=['orange','green','yellow'],
                        choices=['orange','green','yellow','red','blue'])
    parser.add_argument('--duration',type=float,default=0,help='Seconds; zero runs until q or Ctrl-C.')
    parser.add_argument('--no-gui',action='store_true')
    parser.add_argument('--image',help='Process one saved image without creating a ROS node.')
    parser.add_argument('--save-dir',default='lab5_runs/color_tuning')
    argv=sys.argv if args is None else ['lab5_color_view',*args]
    options=parser.parse_args(remove_ros_args(args=argv)[1:])
    vision=load_vision(options.config)
    directory=Path(options.save_dir)
    directory.mkdir(parents=True,exist_ok=True)
    latest={}
    history=deque(maxlen=600)
    frame_count=0
    totals={color:Counter() for color in options.colors}
    def process(image):
        nonlocal frame_count
        frame_count+=1
        overlay,masks,centers=annotate(image,options.colors,vision)
        latest.update(image=image,overlay=overlay,masks=masks,centers=centers)
        history.append(dict(counts={color:len(points) for color,points in centers.items()},centers=centers))
        for color,points in centers.items():
            totals[color][len(points)]+=1
        return overlay,masks,centers
    def save():
        if not latest:
            return
        cv2.imwrite(str(directory/'camera.png'),latest['image'])
        cv2.imwrite(str(directory/'centroids.png'),latest['overlay'])
        for color,mask in latest['masks'].items():
            cv2.imwrite(str(directory/(color+'_mask.png')),mask)
        histogram={color:dict(totals[color]) for color in options.colors}
        report=dict(time_utc=datetime.now(timezone.utc).isoformat(),frames=frame_count,
                    vision=vision,count_histogram=histogram,last_centers=latest['centers'],
                    image_subscription_only=True,arm_commands_sent=0,suction_commands_sent=0,
                    observations=list(history))
        (directory/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Count histogram:',histogram)
        print('Last centers (column,row):',latest['centers'])
        print('Saved color-only evidence:',directory.resolve())
    if options.image:
        process(cv2.imread(options.image))
        save()
        return
    rclpy.init(args=args)
    node=Node('lab5_color_view_only')
    bridge=CvBridge()
    show=not options.no_gui and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    stopped=False
    last_report=0.0
    def callback(msg):
        nonlocal stopped,last_report
        image=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        overlay,masks,centers=process(image)
        if show:
            cv2.imshow('Lab5 COLOR ONLY - no robot commands',overlay)
            for color,mask in masks.items():
                cv2.imshow('Lab5 COLOR ONLY - '+color,mask)
            if (cv2.waitKey(1)&0xff) in [ord('q'),27]:
                stopped=True
        if time.monotonic()-last_report>1:
            print({color:len(points) for color,points in centers.items()},flush=True)
            last_report=time.monotonic()
    subscription=node.create_subscription(Image,'/image_raw',callback,qos_profile_sensor_data)
    start=time.monotonic()
    print('IMAGE SUBSCRIBER ONLY. q/Esc or Ctrl-C exits. No arm publisher or IO client exists.',flush=True)
    try:
        while rclpy.ok() and not stopped and (options.duration<=0 or time.monotonic()-start<options.duration):
            rclpy.spin_once(node,timeout_sec=.1)
            if not history and time.monotonic()-start>8:
                raise RuntimeError('No camera frames received; check camera driver and ROS domain.')
    except KeyboardInterrupt:
        pass
    finally:
        save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if show:
            cv2.destroyAllWindows()


if __name__=='__main__':
    main()
