#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 15:20:41 2022

@author: paulg
"""
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_from_matrix
from ens_vision import tf_tools


def svd_transform(points_A_init,points_B_init):

    points_A = points_A_init.reshape((len(points_A_init),3,1))
    points_B = points_B_init.reshape((len(points_B_init),3,1))


    points_A_barycentre = np.mean(points_A,axis=0)
    points_B_barycentre = np.mean(points_B,axis=0)

    points_A_centered = points_A-points_A_barycentre
    points_B_centered = points_B-points_B_barycentre

    # Compute the covariance matrix
    H = np.zeros((3,3))
    for i in range(len(points_A)):
        H+=np.dot(points_A_centered[i],np.transpose(points_B_centered[i]))


    # SVD decomposition
    U,S,V = np.linalg.svd(H)

    V=np.transpose(V)

    # rotation matrix
    R=np.dot(V,np.transpose(U))
    # print(R)
    if(np.linalg.det(R)<0):
        # print(V)
        V_p=np.copy(V)
        V_p[:,2]=-V[:,2]
        # print(V_p)
        R=np.dot(V_p,np.transpose(U))

    # translation matrix
    T = points_B_barycentre-np.dot(R,points_A_barycentre)


    points_A_in_B = np.transpose(np.dot(R,np.transpose(points_A_init))+T)

    return (R,T,points_A_in_B)





frame_0='marker_0'
frame_1='marker_1'
frame_2='marker_2'

map_frame_id='map'
map_name = 'map5'

number_of_points = 6


def callback(msg):
    global i

    x=msg.point.x
    y=msg.point.y
    z=0

    points_map[i%number_of_points]=[x,y,z]
    print(points_map)

    points_camera[i%number_of_points] = tl.lookupTransform('camera', f'marker_{i}', rospy.Time())[0]
    print(points_camera)

    if i%number_of_points==(number_of_points-1):

        R,T,points_map_in_camera = svd_transform(points_map, points_camera)

        pos = T

        # Compute the rotation matrix
        rotation_matrix = np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 1]],
                                   dtype=float)
        rotation_matrix[:3, :3] = R
        quat = quaternion_from_matrix(rotation_matrix)


        # Compute the error of the transformation
        check_svd = points_camera - points_map_in_camera

        error = np.sqrt(check_svd[:,0]**2+check_svd[:,1]**2+check_svd[:,2]**2)

        print('error for each point in meters :', error)



        tf_tools.save_tf(pos,quat,'map','camera',name=map_name)

        tf_tools.publish_tf(pos, quat, 'map', 'camera',name=map_name, rate=100)



    i+=1
    rospy.loginfo(f'Pick the marker_{i%number_of_points}')


if __name__ == '__main__':
    i=0
    points_camera=np.zeros((number_of_points,3))
    points_map=np.zeros((number_of_points,3))

    rospy.init_node('svd')
    tl = tf.TransformListener()

    try:
        rospy.Subscriber("clicked_point", PointStamped, callback)
        rospy.loginfo(f'Pick the marker_{i}')
        rospy.spin()


    except rospy.ROSInterruptException:

            pass




