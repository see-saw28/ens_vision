#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 09:14:39 2022

@author: student
"""
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
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion, Twist, PoseStamped
from std_msgs.msg import Header, ColorRGBA, String
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_matrix, quaternion_from_euler,euler_from_quaternion
import matplotlib.cm as cm
import numpy as np
import path_tools
import copy


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

class LapPath:

    def __init__(self, name, static_frame='map', moving_frame='base_link',path=Path()):
        self.count = 0 
        self.name = name
        self.static_frame = static_frame
        self.moving_frame = moving_frame
        
        self.lap_publisher = rospy.Publisher(name, Path, queue_size=5)

        self.rate = rospy.Rate(90)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        
        if len(path.poses)>0:
            self.path = path
        else :
            self.path = self.init_path()
            
        self.time = rospy.Time.now().to_sec()
        
 
    def init_path(self):
        path = Path()
        path.header.frame_id = self.static_frame
        
        return path
            
            
   
    
    def init_lap_path(self,name='path'):
        
        self.path = self.init_path()
        self.name = name
        self.lap_publisher = rospy.Publisher(name, Path, queue_size=5)
        
        return self
        
    def rename(self,name):
        self.name = name
        self.lap_publisher = rospy.Publisher(name, Path, queue_size=5)
        
        return self
        
    def distance(self):
        i=0
        distance=0
        for pose in self.path.poses:
            if (i==0):
                old_x=pose.pose.position.x
                old_y=pose.pose.position.y
                old_z=pose.pose.position.z
                
            else:
                x=pose.pose.position.x
                y=pose.pose.position.y
                z=pose.pose.position.z
                
                dist=np.sqrt((x-old_x)**2+(y-old_y)**2+(z-old_z)**2)
                distance+=dist
                
                old_x=x
                old_y=y
                old_z=z
            i+=1
            
        return distance

    def update(self, pose):
        
        pose.header.seq = len(self.path.poses)+1
        self.path.poses.append(pose)
       
        self.publish()
        
    def publish(self):
        
        self.lap_publisher.publish(self.path)
       

class TrajectoryPath:

    def __init__(self, static_frame='map', moving_frame='base_link', moving_frame1='marker1',doubleAruco=False):
        self.lap_count = 0 
        self.static_frame = static_frame
        self.moving_frame = moving_frame
        self.moving_frame1 = moving_frame1
        self.doubleAruco = doubleAruco
        # rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
        self.traj_path = LapPath('trajectory',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.traj_loaded_path = LapPath('trajectory_loaded',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.current_lap_path = LapPath('current_lap_path',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.testing_lap_path = LapPath('testing_lap_path',static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.laps_path = []
        self.command_subscriber = rospy.Subscriber("syscommand", String, self.callback)
        self.ds4_cmd = rospy.Subscriber("cmd_vel", Twist, self.ds4_callback)
        self.rate = rospy.Rate(30)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        self.time = rospy.Time.now().to_sec()
        self.cmd_speed = 0
        self.errors_crosstrack_ext = []
        self.errors_yaw_ext = []
        self.calculate_error = True
        
       
    def ds4_callback(self, msg):
        self.cmd_speed = msg.linear.x
        


    def callback(self,msg):
        # print(msg)
        msg=msg.data.split(" ")
        print(msg)
        if(msg[0]=="reset"):
            self.traj_path.init_lap_path()
            self.current_lap_path.init_lap_path()
            if(len(msg)>1 and msg[1]=="all"):
            
                self.laps=[]
                self.errors_crosstrack_ext = []
                self.errors_yaw_ext = []
                self.lap_count = 0
            
        elif (msg[0]=="save"):
            if len(msg)>1:
                if msg[1]=='all':
                    path_tools.save_path(self.traj_path.path) 
                    
                elif msg[1]=='last':
                    
                    path_tools.save_path(self.laps_path[-1].path)
                    
                elif msg[1]=='test':
                    if len(msg)>2:
                        name = msg[2]
                        path_tools.save_path(self.testing_lap_path.path,name)
                        
                    else :
                        path_tools.save_path(self.testing_lap_path.path)
                elif msg[1]=='error':
                    if len(msg)>2:
                        name = msg[2]
                        path_tools.save_error(self.errors_crosstrack_ext, self.errors_yaw_ext ,name)
                        
                    else :
                        path_tools.save_error(self.errors_crosstrack_ext, self.errors_yaw_ext) 
                else :
                    try :
                        lap_number = int(msg[1])
                        path_tools.save_path(self.laps_path[lap_number-1].path)
                    except :
                        print('Wrong number')
                    
        elif (msg[0]=="load"):
            if len(msg)>1:    
                path = path_tools.load_path(msg[1])
                self.traj_loaded_path = LapPath('trajectory_loaded',static_frame=self.static_frame,moving_frame=self.moving_frame,path=path)

                path_x = []
                path_y = []
                path_yaw = []
                
                for i, pose in enumerate(path.poses):
                    path_x.append(pose.pose.position.x)
                    path_y.append(pose.pose.position.y)
                    orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                    _, _, yaw = euler_from_quaternion(orientation_list)
                    path_yaw.append(yaw)
                
                self.course_x = path_x
                self.course_y = path_y
                self.course_yaw = path_yaw
            
        elif (msg[0]=="lap"):
            # current_time=rospy.Time.now().to_sec()
            # lap_time = current_time - self.time
            # self.time = current_time
            # print('Lap time:' ,lap_time)
            path=copy.deepcopy(self.current_lap_path.path)
            
            self.lap_count += 1
            self.current_lap_path.init_lap_path('current_lap_path')
            
            self.laps_path.append(LapPath(f'lap_{len(self.laps_path)+1}_path',static_frame=self.static_frame,moving_frame=self.moving_frame,path=path))
            lap_time=self.current_lap_path.path.header.stamp.to_sec()-self.laps_path[-1].path.header.stamp.to_sec()
            # print(f'Lap {len(self.laps_path)}: time: {lap_time:.2f}s & distance: {self.laps_path[-1].distance():.2f}m')
            
    def normalize_angle(self, angle):
        # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle           
           
    def calc_error(self, x, y, yaw, course_x, course_y, course_yaw):
        # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

        

        # Search nearest point index
        dx = [x - icx for icx in course_x]
        dy = [y - icy for icy in course_y]
        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        error_front_axle = min(d)
        target_idx = d.index(error_front_axle)
        # print(target_idx)

        error_yaw = self.normalize_angle(course_yaw[target_idx] - yaw)
            
        
        return error_front_axle, error_yaw       


    def update(self):
        pos, quat = self.tl.lookupTransform(self.static_frame,self.moving_frame,rospy.Time())
        lin, ang = self.tl.lookupTwist(self.moving_frame, self.static_frame, rospy.Time(), rospy.Duration(0.1))
        vit=np.linalg.norm(lin)
        
        pose = PoseStamped()
        
        pose.header.frame_id = self.static_frame
        
        
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        
        if self.doubleAruco :
            x1, y1, z1 = self.tl.lookupTransform(self.static_frame, self.moving_frame1, rospy.Time())[0]
            
            yaw = np.arctan2(y1 - pos[1], x1 - pos[0])
            
            quat = quaternion_from_euler(0,0,yaw)
            
        else :
            _,_,yaw = euler_from_quaternion(**quat)
        
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        
        self.traj_path.update(pose)
        self.current_lap_path.update(pose)
        if self.lap_count > 0 and  self.lap_count < 6 :
            self.testing_lap_path.update(pose)
            
            if self.calculate_error :
                error_crosstrack_ext, error_yaw_ext = self.calc_error(pos[0], pos[1], yaw ,self.course_x, self.course_y, self.course_yaw)
                
                self.errors_crosstrack_ext.append(error_crosstrack_ext)
                self.errors_yaw_ext.append(error_yaw_ext)
                # print(len(self.errors_crosstrack_ext))
        self.traj_loaded_path.publish()
         
        for lap in self.laps_path:
            lap.publish()
        
        # rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_path_node", anonymous=False)
    rospack = rospkg.RosPack()
    if (len(sys.argv)>2):
        static_frame_id=sys.argv[1]
        moving_frame_id=sys.argv[2]
        trajectory_interactive_path = TrajectoryPath(static_frame=static_frame_id, moving_frame=moving_frame_id)
    else :
        
        trajectory_interactive_path = TrajectoryPath(static_frame='map', moving_frame='marker_0', moving_frame1='marker_1',doubleAruco=True)
    while not rospy.is_shutdown():
        try:

            trajectory_interactive_path.update()

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')
            
            continue

       
        trajectory_interactive_path.rate.sleep()