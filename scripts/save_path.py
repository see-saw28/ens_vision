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

def header_to_numpy(header):
    return np.array([header.seq, [header.stamp.secs, header.stamp.nsecs], header.frame_id], dtype=object)

def numpy_to_header(array):
    header=Header()
    header.seq=array[0]
    header.stamp.secs=array[1][0]
    header.stamp.nsecs=array[1][1]
    header.frame_id=array[2]
    return header
    
def path_to_numpy(msg):
    np_header=header_to_numpy(msg.header)
    np_path=np.empty(len(msg.poses)+1, dtype=object)
    print(np_header)
    np_path[0]=np_header
    i=0
    for pose in msg.poses:
        i+=1
        #print(pose)
        np_pose = to_numpy(pose)
        #print('np_pose',type(np_pose[1]))
        #to_message(Pose, np_pose)
        np_header = header_to_numpy(pose.header)
        np_path[i] = [np_header, np_pose]
    return np_path 
    
    
def numpy_to_path(np_path):
    msg=Path()
    msg.header=np_path[0]
    poses=[]
    for i in range (1,len(np_path)):
        pose=PoseStamped()
        pose.header=numpy_to_header(np_path[i][0])
        pose.pose=to_message(Pose,np_path[i][1])
        break
        poses.append[pose]
    return msg


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
    filename=check_file('paths/path.pckl')
    print(filename)
    f = open(filename, 'wb')
    
    pickle.dump(msg, f)
    f.close()
    print('Path saved with :',len(msg.points), 'poses')
    
    rate.sleep()

    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
    
          
            
            

if __name__ == '__main__':

    rospy.init_node('path_saver')
    rate = rospy.Rate(0.05)
    try:
        rospy.Subscriber("/path_marker", Marker, callback)
        rospy.spin()
        
    except rospy.ROSInterruptException:
       
        pass
