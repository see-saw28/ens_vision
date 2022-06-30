#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 13 21:08:12 2022

@author: paul
"""



import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

import tf


from numpy_ros import to_numpy, to_message

# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys
import pyrealsense2 as rs
import os
import pickle
import rospkg
rospack = rospkg.RosPack()
import path_tools

def path(filename='path'):
    pub = rospy.Publisher(f'trajectory_{filename}', Path, queue_size=10)
    rospy.init_node('ros_path', anonymous=True)
    rate = rospy.Rate(10)

    if 'mcp' in filename :
        mcp = path_tools.load_mcp(filename)
        path = path_tools.mcp_to_path(mcp)
    else :
        path = path_tools.load_path(filename)



    while not rospy.is_shutdown():



        pub.publish(path)
        print(len(path.poses))
        rate.sleep()

if __name__ == '__main__':
    try:
        if (len(sys.argv)>1):
            filename=sys.argv[1]
        else :
            filename='test_aruco_18_traj'

        path(filename)
    except rospy.ROSInterruptException:
        pass






