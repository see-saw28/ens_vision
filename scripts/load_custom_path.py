#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 23 16:31:46 2022

@author: student
"""
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

def path(filename='path'):
    pub = rospy.Publisher('path_marker', Marker, queue_size=10)
    rospy.init_node('ros_path', anonymous=False)
    rate = rospy.Rate(10)

	
	
    obj = Path()

    while not rospy.is_shutdown():

        f = open(f'paths/{filename}.pckl', 'rb')
        obj = pickle.load(f)
        f.close()

        pub.publish(obj)
        print(len(obj.points))
        rate.sleep()

if __name__ == '__main__':
    try:
        if (len(sys.argv)>1):
            filename=sys.argv[1]
           
        path(filename)
    except rospy.ROSInterruptException:
        pass






