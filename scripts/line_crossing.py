#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 17 19:14:14 2022

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
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String

...
# get twist relative to map calculated from the most recent
# valid sample to a sample 0.5 seconds before that

def dot(A,B):
    return (A.real*B.real+A.imag*B.imag)

def cros(A,B):
    return (A.real*B.imag-A.imag*B.real)

def navigation():
    static_frame_id="map"
    moving_frame_id="marker_0"
    
    # static_frame_id="camera_odom_frame"
    # moving_frame_id="camera_pose_frame"
    
    
    
    
    if (len(sys.argv)>2):
        static_frame_id=sys.argv[1]
        moving_frame_id=sys.argv[2]
        
    
    pub_freq = 30.0
    rospy.init_node('line_crossing', anonymous=False)
    # pub = rospy.Publisher(f'vel_from_tf/{moving_frame_id}', Twist, queue_size=2) 
    rate = rospy.Rate(pub_freq)
    tl = tf.TransformListener()
    
    xa=0
    ya=1.9
    A=complex(xa,ya)
    
    xb=0
    yb=2.84
    
    B=complex(xb,yb)
    AB=A-B
    BA=-AB
    
    
    
    marker_publisher = rospy.Publisher('finish_line', Marker, queue_size=5)
    marker = Marker()
    marker.type=Marker.LINE_STRIP
    marker.lifetime=rospy.Duration(100)
    marker.scale=Vector3(0.1, 0.01, 0.01)
    marker.header=Header(frame_id=static_frame_id)
    marker.pose=Pose(Point(0,0,0), Quaternion(0,0,0,1))
    
    
    marker.points=[Point(xa,ya,0),Point(xb,yb,0)]
    couleur=ColorRGBA(0,0,1,1)
    marker.colors=[couleur,couleur]
    
    marker_publisher.publish(marker)
    
    old_ABxAP=0
    pub = rospy.Publisher('syscommand', String, queue_size=10)
    
    print('a')
    
    while not rospy.is_shutdown():
        try:

            pos, quat = tl.lookupTransform(static_frame_id,moving_frame_id,rospy.Time())
            
            xp, yp,_=pos
            
            P=complex(xp,yp)
            
            
            AP=A-P
            BP=B-P
            
            if(dot(AB,AP)>0 and dot(BA,BP)>0):
                
                if (cros(AB,AP)*old_ABxAP<0):
                    # print('Line crossed')
                    msg=String()
                    msg.data='lap'
                    pub.publish(msg)
                    
                
            old_ABxAP=cros(AB,AP)

            
            marker_publisher.publish(marker)

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

        
        rate.sleep()

if __name__ == '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
