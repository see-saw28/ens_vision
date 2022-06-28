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

def path(filename='path'):
    pub = rospy.Publisher('trajectory', Path, queue_size=10)
    rospy.init_node('ros_path', anonymous=False)
    rate = rospy.Rate(10)



    obj = Path()

    while not rospy.is_shutdown():

        f = open(rospack.get_path('ens_vision')+f'/tests/{filename}.pckl', 'rb')
        obj = pickle.load(f)
        f.close()

        pub.publish(obj)
        print(len(obj.poses))
        rate.sleep()

if __name__ == '__main__':
    try:
        if (len(sys.argv)>1):
            filename=sys.argv[1]
        else :
            filename='test_aruco_18'

        path(filename)
    except rospy.ROSInterruptException:
        pass






