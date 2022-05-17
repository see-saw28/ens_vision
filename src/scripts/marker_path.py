#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 17 19:14:14 2022

@author: paul
"""
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String

import matplotlib.cm as cm
import numpy as np


class TrajectoryInteractiveMarkers:

    def __init__(self):
        self.count = 0 
        # rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.rate=rate = rospy.Rate(15)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        self.marker = Marker()        
        
        self.marker.type=Marker.LINE_STRIP
        self.marker.lifetime=rospy.Duration(100)
        self.marker.scale=Vector3(0.01, 0.01, 0.01)
        self.marker.header=Header(frame_id='marker_0')
        self.marker.pose=Pose(Point(0,0,0), Quaternion(0,0,0,1))

    def event_in_cb(self,msg):
        self.waypoints = msg
        self.a = [1, 1, 1]
        #self.a = list()
        #self.a.append(self.waypoints.twist.linear.x)
        #self.a.append(self.waypoints.twist.linear.y)
        #self.a.append(self.waypoints.twist.linear.z)
        
        self.marker()

    def marker_from_tf(self):
        pos, quat = self.tl.lookupTransform('marker_0','marker_2',rospy.Time())
        lin, ang = self.tl.lookupTwist('marker_2', 'marker_0', rospy.Time(), rospy.Duration(0.1))
        vit=np.linalg.norm(lin)
        
        
        self.marker.points.append(Point(*pos))
        
        self.marker.header=Header(frame_id='marker_0')
        self.marker.color=ColorRGBA(*cm.jet(vit))
        self.marker.colors.append(ColorRGBA(*cm.jet(vit)))
        self.marker_publisher.publish(self.marker)
        
        rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    while not rospy.is_shutdown():
        try:

            trajectory_interactive_markers.marker_from_tf()

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

       
        trajectory_interactive_markers.rate.sleep()