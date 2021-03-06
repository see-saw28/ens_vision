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

    def __init__(self, static_frame='map', moving_frame='base_link'):
        self.count = 0 
        self.static_frame = static_frame
        self.moving_frame = moving_frame
        # rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
        self.marker_publisher = rospy.Publisher('path_marker', Marker, queue_size=5)
        self.current_lap_publisher = rospy.Publisher('current_lap_marker', Marker, queue_size=5)
        self.laps_publisher=[]
        self.command_subscriber = rospy.Subscriber("syscommand", String, self.callback)
        self.rate=rate = rospy.Rate(15)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        self.marker = self.init_marker()
        self.lap_marker = self.init_marker()
        self.laps=[]
        self.time = rospy.Time.now().to_sec()
       
        


    def callback(self,msg):
        msg=msg.data.split(" ")
        if(msg[0]=="reset"):
            self.marker = self.init_marker()
            
            if(len(msg)>1 and msg[1]=="all"):
            
                self.laps=[]
            
        elif (msg[0]=="save"):
            print('path saved')
            
        elif (msg[0]=="lap"):
            # current_time=rospy.Time.now().to_sec()
            # lap_time = current_time - self.time
            # self.time = current_time
            # print('Lap time:' ,lap_time)
            self.laps.append(self.lap_marker)
            self.lap_marker=self.init_marker()
            self.laps_publisher.append(rospy.Publisher(f'lap_{len(self.laps)}_marker', Marker, queue_size=5))
            lap_time=self.lap_marker.header.stamp.to_sec()-self.laps[-1].header.stamp.to_sec()
            print(f'Lap {len(self.laps)}: time: {lap_time:.2f}s & distance: {self.distance(self.laps[-1]):.2f}m')
           
            
            
    def init_marker(self):
        marker = Marker()     
        marker.header=Header(frame_id=self.static_frame,stamp=rospy.Time.now())
        marker.type=Marker.LINE_STRIP
        marker.lifetime=rospy.Duration(100)
        marker.scale=Vector3(0.01, 0.01, 0.01)
        marker.pose=Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        return marker
        
    def distance(self, msg):
        i=0
        distance=0
        for point in msg.points:
            if (i==0):
                old_x=point.x
                old_y=point.y
                old_z=point.z
                
            else:
                x=point.x
                y=point.y
                z=point.z
                
                dist=np.sqrt((x-old_x)**2+(y-old_y)**2+(z-old_z)**2)
                distance+=dist
                
                old_x=x
                old_y=y
                old_z=z
            i+=1
            
        return distance

    def marker_from_tf(self):
        pos, quat = self.tl.lookupTransform(self.static_frame,self.moving_frame,rospy.Time())
        lin, ang = self.tl.lookupTwist(self.moving_frame, self.static_frame, rospy.Time(), rospy.Duration(0.1))
        vit=np.linalg.norm(lin)
        
        
        self.marker.points.append(Point(*pos))
        self.marker.colors.append(ColorRGBA(*cm.turbo(vit/4)))
        
        self.lap_marker.points.append(Point(*pos))
        self.lap_marker.colors.append(ColorRGBA(*cm.turbo(vit/4)))
        
        self.marker_publisher.publish(self.marker)
        self.current_lap_publisher.publish(self.lap_marker)
        
        for i, publisher in enumerate(self.laps_publisher):
            publisher.publish(self.laps[i])
        
        # rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
    trajectory_interactive_markers = TrajectoryInteractiveMarkers(static_frame='camera_odom_frame', moving_frame='camera_pose_frame')
    while not rospy.is_shutdown():
        try:

            trajectory_interactive_markers.marker_from_tf()

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

       
        trajectory_interactive_markers.rate.sleep()