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

        self.init_lap_path()

        self.rate = rospy.Rate(90)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)

        if len(path.poses)>0:
            self.path = path


        self.time = rospy.Time.now().to_sec()


    def init_path(self):
        path = Path()
        path.header.frame_id = self.static_frame
        path.header.stamp = rospy.Time.now()

        return path




    def init_lap_path(self):

        self.path = self.init_path()

        self.lap_publisher = rospy.Publisher(self.name, Path, queue_size=5)

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

        if moving_frame == 'base_link':
            self.suffix = '_amcl'
        else :
            self.suffix = '_aruco'
        self.doubleAruco = doubleAruco
        # rospy.Subscriber("/arm_1/arm_controller/cartesian_velocity_command",TwistStamped, self.event_in_cb)
        self.traj_path = LapPath('trajectory'+self.suffix,static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.traj_loaded_path = LapPath('trajectory_loaded'+self.suffix,static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.current_lap_path = LapPath('current_lap_path'+self.suffix,static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.testing_lap_path = LapPath('testing_lap_path'+self.suffix,static_frame=self.static_frame,moving_frame=self.moving_frame)
        self.laps_time = [rospy.Time.now().to_sec()]
        self.command_subscriber = rospy.Subscriber("syscommand", String, self.callback)
        self.ds4_cmd = rospy.Subscriber("cmd_vel", Twist, self.ds4_callback)
        self.rate = rospy.Rate(30)
        self.tl = tf.TransformListener()
        rospy.sleep(0.5)
        self.time = rospy.Time.now().to_sec()
        self.cmd_speed = 0
        self.number_laps = 5


    def ds4_callback(self, msg):
        self.cmd_speed = msg.linear.x



    def callback(self,msg):
        # print(msg)
        msg=msg.data.split(" ")
        print(msg)
        if(msg[0]=="reset"):
            self.traj_path.init_lap_path()
            self.current_lap_path.init_lap_path()
            self.testing_lap_path.init_lap_path()
            if(len(msg)>1 and msg[1]=="all"):

                self.laps_time=[rospy.Time.now().to_sec()]
                self.lap_count = 0

        elif (msg[0]=="save"):
            if len(msg)>1:
                if msg[1]=='all':
                    path_tools.save_path(self.traj_path.path)

                elif msg[1]=='last':

                    path_tools.save_path(self.laps_path[-1].path, self.suffix)

                elif msg[1]=='test':
                    if len(msg)>2:
                        name = msg[2]
                        path_tools.save_path(self.testing_lap_path.path,name)

                    else :
                        path_tools.save_path(self.testing_lap_path.path,self.suffix)

                else :
                    try :
                        lap_number = int(msg[1])
                        path_tools.save_path(self.laps_path[lap_number-1].path, self.suffix)
                    except :
                        print('Wrong number')

        elif (msg[0]=="load"):
            if len(msg)>1:
                if 'mcp' in msg[1]:
                    mcp = path_tools.load_mcp(msg[1])
                    path = path_tools.mcp_to_path(mcp)
                else :
                    path = path_tools.load_path(msg[1])
                self.traj_loaded_path = LapPath('trajectory_loaded'+self.suffix,static_frame=self.static_frame,moving_frame=self.moving_frame,path=path)
                self.path_name = msg[1]



        elif (msg[0]=="lap"):
            # current_time=rospy.Time.now().to_sec()
            # lap_time = current_time - self.time
            # self.time = current_time
            # print('Lap time:' ,lap_time)

            self.laps_time.append(rospy.Time.now().to_sec())
            self.lap_count += 1



            lap_time=self.laps_time[-1]-self.laps_time[-2]
            print(f'Lap {self.lap_count}: time: {lap_time:.2f}s')

            if self.lap_count == self.number_laps + 1:
                total_lap_time = self.laps_time[-1] - self.laps_time[1]

                print(total_lap_time)

                path_tools.save_test(self.testing_lap_path.path, name='test'+self.suffix, ref_path_name=self.path_name, ref_path=None, map_name='map5', total_laps = self.number_laps, total_time = total_lap_time, laps_time = self.laps_time)



    def update(self):
        pos, quat = self.tl.lookupTransform(self.static_frame,self.moving_frame,rospy.Time())

        pose = PoseStamped()

        pose.header.frame_id = self.static_frame
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]

        if self.doubleAruco :
            x1, y1, z1 = self.tl.lookupTransform(self.static_frame, self.moving_frame1, rospy.Time())[0]

            yaw = np.arctan2(y1 - pos[1], x1 - pos[0])

            quat = quaternion_from_euler(0,0,yaw)
            _,_,yaw = euler_from_quaternion(quat)

        else :
            _,_,yaw = euler_from_quaternion(quat)

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]


        # self.traj_path.update(pose)
        # self.current_lap_path.update(pose)
        if self.lap_count > 0 and  self.lap_count < self.number_laps + 1 :
            self.testing_lap_path.update(pose)


        # self.traj_loaded_path.publish()

        # for lap in self.laps_path:
        #     lap.publish()

        # rospy.loginfo('msg published')

if __name__ == '__main__':
    rospy.init_node("trajectory_path_node", anonymous=False)
    rospack = rospkg.RosPack()
    if (len(sys.argv)>2):
        static_frame_id=sys.argv[1]
        moving_frame_id=sys.argv[2]
        trajectory_interactive_path = TrajectoryPath(static_frame=static_frame_id, moving_frame=moving_frame_id)
    else :

        trajectory_aruco = TrajectoryPath(static_frame='map', moving_frame='marker_0', moving_frame1='marker_1',doubleAruco=True)
        trajectory_amcl = TrajectoryPath(static_frame='map', moving_frame='base_link', doubleAruco=False)
    while not rospy.is_shutdown():
        try:

            trajectory_aruco.update()
            trajectory_amcl.update()


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # print('looser')

            continue


        trajectory_aruco.rate.sleep()

