#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 23 16:28:05 2022

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



def check_file(filePath):
    if os.path.exists(filePath):
        numb = 1
        while True:
            newPath = "{0}_{2}{1}".format(*os.path.splitext(filePath) + (numb,))
            if os.path.exists(newPath):
                numb += 1
            else:
                return newPath
    return filePath  


def callback(msg):
    filename=check_file('paths/custom_path.pckl')
    print(filename)
    f = open(filename, 'wb')
    
    pickle.dump(msg, f)
    f.close()
    print('Path saved with :',len(msg.points), 'poses')
    
    rate.sleep()

    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
    
          
            
            

if __name__ == '__main__':

    rospy.init_node('custom_path_saver')
    rate = rospy.Rate(0.05)
    try:
        rospy.Subscriber("/path_marker", Marker, callback)
        rospy.spin()
        
    except rospy.ROSInterruptException:
       
        pass
