#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 18 11:31:37 2022

@author: student
"""


import time
import rospy
import numpy as np
import os
import sys



# os.environ['ROS_MASTER_URI']='http://172.20.10.9:11311'
# os.environ['ROS_IP']='172.20.10.8'


import tf
from geometry_msgs.msg import Twist    
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped 
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
        

if __name__ == '__main__':
    marker_frame_id="marker_0"
    map_frame_id="marker_1"
    
    if (len(sys.argv)>2):
        marker_frame_id=sys.argv[1]
        map_frame_id=sys.argv[2]
        
    
    rospy.init_node('vel_from_tf', anonymous=False)
    
    tl = tf.TransformListener()
    while not rospy.is_shutdown():
       
        try:
        
            
            pos,quat = tl.lookupTransform(marker_frame_id, map_frame_id, rospy.Time())
            filename=check_file('tf.pckl')
            print(filename)
            f = open(filename, 'wb')
            pickle.dump([pos,quat,map_frame_id, marker_frame_id], f)
            f.close()
            print('Tf saved')
            break
            
            
        
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('no tf found')
        
        rospy.Rate(10).sleep()
