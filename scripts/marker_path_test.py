#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 17 19:14:14 2022

@author: paul
"""
import rospy
import tf
import pickle
import os
import rospkg
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion, Twist
from std_msgs.msg import Header, ColorRGBA, String

import matplotlib.cm as cm
import numpy as np


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

class LapMarkers:

    def __init__(self, name, static_frame='map', moving_frame='base_link',marker=Marker(),speeds=[],orientations=[],cmd_speeds=[]):
        self.count = 0 
        self.name = name
        self.static_frame = static_frame
        self.moving_frame = moving_frame
        
        self.lap_publisher = rospy.Publisher(name, Marker, queue_size=5)

        self.rate= rospy.Rate(90)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        
        if len(marker.points)==0:
            self.marker = self.init_marker()
        else:
            self.marker = marker
            
        self.time = rospy.Time.now().to_sec()
        if len(speeds)==0:
            self.speeds=[]
            self.orientations=[]
            self.cmd_speeds=[]
        else:
            self.speeds = speeds
            self.orientations = orientations
            self.cmd_speeds=cmd_speeds
 
            
            
    def init_marker(self):
        marker = Marker()     
        marker.header=Header(frame_id=self.static_frame,stamp=rospy.Time.now())
        marker.type=Marker.LINE_STRIP
        marker.lifetime=rospy.Duration(100)
        marker.scale=Vector3(0.01, 0.01, 0.01)
        marker.pose=Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        return marker
    
    def init_lap_marker(self,name='marker'):
        
        self.marker = self.init_marker()
        self.name = name
        self.lap_publisher = rospy.Publisher(name, Marker, queue_size=5)
        
        return self
        
    def rename(self,name):
        self.name = name
        self.lap_publisher = rospy.Publisher(name, Marker, queue_size=5)
        
        return self
        
    def distance(self):
        i=0
        distance=0
        for point in self.marker.points:
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

    def update(self,cmd_speed):
        pos, quat = self.tl.lookupTransform(self.static_frame,self.moving_frame,rospy.Time())
        lin, ang = self.tl.lookupTwist(self.moving_frame, self.static_frame, rospy.Time(), rospy.Duration(0.1))
        vit=np.linalg.norm(lin)
        
        
        self.marker.points.append(Point(*pos))
        self.marker.colors.append(ColorRGBA(*cm.turbo(vit/4)))
        self.speeds.append(vit)
        self.orientations.append(quat)
        self.cmd_speeds.append(cmd_speed)
        self.publish()
        
    def publish(self):
        
        self.lap_publisher.publish(self.marker)
       

class TrajectoryMarkers:

    def __init__(self, static_frame='map', moving_frame='base_link'):
        self.count = 0 
        self.static_frame = static_frame
        self.moving_frame = moving_frame
        # rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
        self.traj_marker = LapMarkers('trajectory_marker',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.traj_loaded_marker = LapMarkers('trajectory_loaded',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.current_lap_marker = LapMarkers('current_lap_marker',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.laps_markers = []
        self.command_subscriber = rospy.Subscriber("syscommand", String, self.callback)
        self.ds4_cmd = rospy.Subscriber("cmd_vel", Twist, self.ds4_callback)
        self.rate = rospy.Rate(30)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        self.time = rospy.Time.now().to_sec()
        self.cmd_speed = 0
        
       
    def ds4_callback(self, msg):
        self.cmd_speed = msg.linear.x
        


    def callback(self,msg):
        # print(msg)
        msg=msg.data.split(" ")
        print(msg)
        if(msg[0]=="reset"):
            self.traj_marker.init_lap_marker()
            self.current_lap_marker.init_lap_marker()
            if(len(msg)>1 and msg[1]=="all"):
            
                self.laps=[]
            
        elif (msg[0]=="save"):
            if len(msg)>1:
                if msg[1]=='all':
                    filename=check_file(rospack.get_path('ens_vision')+'/paths/traj.pckl')
                    print(filename)
                    f = open(filename, 'wb')
                    
                    pickle.dump([self.traj_marker.marker,self.traj_marker.speeds,self.traj_marker.orientations,self.traj_marker.cmd_speeds], f)
                    f.close()
                    print('Path saved with :',len(self.traj_marker.marker.points), 'poses') 
                    
                elif msg[1]=='last':
                    
                    
                    filename=check_file(rospack.get_path('ens_vision')+'/paths/traj.pckl')
                    print(filename)
                    f = open(filename, 'wb')
                    
                    pickle.dump([self.laps_markers[-1].marker,self.laps_markers[-1].speeds,self.laps_markers[-1].orientations,self.laps_markers[-1].cmd_speeds], f)
                    f.close()
                    print('Path saved with :',len(self.laps_markers[-1].marker.points), 'poses') 
                    
        elif (msg[0]=="load"):
            if len(msg)>1:    
                f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
                marker,speeds,orientations,cmd_speeds = pickle.load(f)
                f.close()
                self.traj_loaded_marker = LapMarkers('trajectory_loaded',static_frame=self.static_frame,moving_frame=self.moving_frame,marker=marker,speeds=speeds,orientations=orientations,cmd_speeds=cmd_speeds)
            
        elif (msg[0]=="lap"):
            # current_time=rospy.Time.now().to_sec()
            # lap_time = current_time - self.time
            # self.time = current_time
            # print('Lap time:' ,lap_time)
            self.laps_markers.append(self.current_lap_marker.rename(f'lap_{len(self.laps_markers)+1}_marker'))
            
            self.current_lap_marker = LapMarkers('current_lap_marker',static_frame=self.static_frame,moving_frame=self.moving_frame)
            lap_time=self.current_lap_marker.marker.header.stamp.to_sec()-self.laps_markers[-1].marker.header.stamp.to_sec()
            print(f'Lap {len(self.laps_markers)}: time: {lap_time:.2f}s & distance: {self.laps_markers[-1].distance():.2f}m, vitesse moyenne = {np.mean(self.laps_markers[-1].cmd_speeds)}')
            
            
           
            


    def update(self):
        
        
        
        self.traj_marker.update(self.cmd_speed)
        self.current_lap_marker.update(self.cmd_speed)
        self.traj_loaded_marker.publish()
         
        for lap in self.laps_markers:
            lap.publish()
        
        # rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_markers_node", anonymous=True)
    rospack = rospkg.RosPack()
    if (len(sys.argv)>2):
        static_frame_id=sys.argv[1]
        moving_frame_id=sys.argv[2]
        trajectory_interactive_markers = TrajectoryMarkers(static_frame=static_frame_id, moving_frame=moving_frame_id)
    else :
        
        trajectory_interactive_markers = TrajectoryMarkers(static_frame='map', moving_frame='base_link')
    while not rospy.is_shutdown():
        try:

            trajectory_interactive_markers.update()

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

       
        trajectory_interactive_markers.rate.sleep()