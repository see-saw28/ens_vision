#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 14 11:48:36 2022

@author: paul
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


...
# get twist relative to map calculated from the most recent
# valid sample to a sample 0.5 seconds before that

def navigation():
    static_frame_id="camera_odom_frame"
    moving_frame_id="camera_pose_frame"
    
    if (len(sys.argv)>2):
        static_frame_id=sys.argv[1]
        moving_frame_id=sys.argv[2]
        
    
    pub_freq = 30.0
    rospy.init_node('vel_from_tf', anonymous=False)
    pub = rospy.Publisher('vel', Twist, queue_size=2) 
    rate = rospy.Rate(pub_freq)
    tl = tf.TransformListener()
    
    while not rospy.is_shutdown():
        try:

            tw = tl.lookupTwistFull(moving_frame_id, static_frame_id,moving_frame_id,(0,0,0),moving_frame_id,rospy.Time(0),rospy.Duration(0.05))
            
            #twist = Twist(Vector3(tw[0][0], tw[0][1], tw[0][2]),
             # Vector3(tw[1][0], tw[1][1], tw[1][2]))
              
            twist = Twist(Vector3(np.sqrt(tw[0][0]**2+ tw[0][1]**2+ tw[0][2]**2),0,0), Vector3(tw[1][0], tw[1][1], tw[1][2]))

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass

	
