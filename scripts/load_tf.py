#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 18 11:42:09 2022

@author: student
"""


import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped 
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf



# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys
import pyrealsense2 as rs
import os
import pickle

def path():
    
    rospy.init_node('ros_path', anonymous=False)
    rate = rospy.Rate(100)

    br = tf.TransformBroadcaster()
    
    f = open('tf.pckl', 'rb')
    pos,quat = pickle.load(f)
    f.close()
	


    while not rospy.is_shutdown():

        
        br.sendTransform(pos,quat,rospy.Time.now(),map_frame_id,marker_frame_id)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        marker_frame_id="marker_0"
        map_frame_id="marker_1"
        
        if (len(sys.argv)>2):
            marker_frame_id=sys.argv[1]
            map_frame_id=sys.argv[2]
        path()
    except rospy.ROSInterruptException:
        pass



    
    
