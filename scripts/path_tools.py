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
import tf
from tf.transformations import quaternion_from_matrix, quaternion_from_euler,euler_from_quaternion

from numpy_ros import to_numpy, to_message

# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys

import os
import pickle
import rospkg
import yaml
rospack = rospkg.RosPack()


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

def save_mcp(traj):

    filename=check_file(rospack.get_path('ens_vision')+'/paths/mcp.npy')

    f = open(filename, 'wb')
    np.save(f, traj)
    f.close()

    print('saved mcp path :', filename)

    return filename


def save_path(path, name='path', test=False):

    if test :
        filename=check_file(rospack.get_path('ens_vision')+f'/tests/{name}.pckl')
    else :
        filename=check_file(rospack.get_path('ens_vision')+f'/paths/{name}.pckl')

    f = open(filename, 'wb')
    pickle.dump(path, f)
    f.close()

    print('saved ROS path :', filename)

    return filename

def save_custom_path(marker,speeds,orientations,cmd_speeds):

    filename=check_file(rospack.get_path('ens_vision')+'/paths/custom_path.pckl')

    f = open(filename, 'wb')
    pickle.dump([marker,speeds,orientations,cmd_speeds], f)
    f.close()

    print('saved custom path :', filename)

    return filename


def load_mcp(name):



    f = open(rospack.get_path('ens_vision')+f'/paths/{name}.npy', 'rb')
    mcp = np.load(f)
    f.close()

    print('load mcp path with :', len(mcp), ' poses')

    return mcp




def load_path(name, absolute_path=False):

    if absolute_path :
        f = open(name, 'rb')
    else :
        f = open(rospack.get_path('ens_vision')+f'/paths/{name}.pckl', 'rb')

    path = pickle.load(f)
    f.close()

    print('load ROS path with :', len(path.poses), ' poses')

    return path



def load_custom_path(name):

    if 'custom_path' in  name :

        f = open(rospack.get_path('ens_vision')+f'/paths/{name}.pckl', 'rb')
        marker,speeds,orientations,cmd_speeds = pickle.load(f)
        f.close()

        print('load custom path with :', len(speeds), ' poses')

        return marker,speeds,orientations,cmd_speeds

    else :
        print('wrong name')
        return None

def mcp_to_path(mcp):
    path = Path()
    path.header.frame_id = 'map'

    for i, position in enumerate(mcp):
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.seq = i

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        quat = quaternion_from_euler(0,0,position[2])

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        path.poses.append(pose)

    return path


def xy_to_path(X,Y):
    path = Path()
    path.header.frame_id = 'map'

    for i in range(len(X)):
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.seq = i

        pose.pose.position.x = X[i]
        pose.pose.position.y = Y[i]

        yaw = np.arctan2(Y[(i+1)%len(Y)]-Y[i], X[(i+1)%len(Y)]-X[i])

        quat = quaternion_from_euler(0,0,yaw)

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        path.poses.append(pose)

    return path

def path_to_xyyaw(path):
    path_x = []
    path_y = []
    path_yaw = []


    for i, pose in enumerate(path.poses):
        path_x.append(pose.pose.position.x)
        path_y.append(pose.pose.position.y)
        orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        path_yaw.append(yaw)

    return path_x, path_y, path_yaw

def save_error(crosstrack,yaw, name='error',test=False):

    if test :
        filename=check_file(rospack.get_path('ens_vision')+f'/tests/{name}.npy')

    else :
        filename=check_file(rospack.get_path('ens_vision')+f'/errors/{name}.npy')

    f = open(filename, 'wb')
    error = np.array([crosstrack,yaw])
    np.save(f, error)
    f.close()

    print('saved error :', filename, f' with {len(crosstrack)} points')

    return filename


def load_error(name, absolute=False, test=False):


    if test :
        f = open(rospack.get_path('ens_vision')+f'/tests/{name}.npy', 'rb')
    else :
        f = open(rospack.get_path('ens_vision')+f'/errors/{name}.npy', 'rb')
    error = np.load(f)
    f.close()

    print('load error with :', len(error[0]), ' poses')

    if absolute :
        return abs(error[0]),abs(error[1])

    else :
        return error[0],error[1]

def save_test(path, name='test', ref_path_name='', ref_path=None, map_name='test_map1_clean', total_laps = 0, total_time = 0, laps_time = []):

    filename=check_file(rospack.get_path('ens_vision')+f'/tests/{name}.yaml')
    name=filename.split('/')[-1].split('.')[0]
    traj_path = save_path(path, name+'_traj',test=True)
    if ref_path != None:

        ref_traj_path = save_path(ref_path, name+'_ref_traj', test=True)
    else :
        ref_traj_path = ''

    driving_mode = rospy.get_param('joy_to_cmd_vel/driving_mode')
    speed = rospy.get_param('joy_to_cmd_vel/max_velocity')

    test_data = {'path_filename':traj_path, 'ref_traj_name':ref_path_name, 'ref_path_filename':ref_traj_path, 'map_name':map_name, 'driving_mode':driving_mode, 'speed':speed, 'total_laps':total_laps,'total_time':total_time, 'lap_time':laps_time}

    filename=check_file(rospack.get_path('ens_vision')+f'/tests/{name}.yaml')

    f = open(filename, 'w')
    yaml.dump(test_data,f, default_flow_style=False)

    f.close()