#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 18 11:31:37 2022

@author: see-saw28
"""



import rospy
import sys
import tf
from ens_vision import tf_tools

if __name__ == '__main__':
    marker_frame_id="marker_0"
    map_frame_id="marker_1"

    if (len(sys.argv)>2):
        marker_frame_id=sys.argv[1]
        map_frame_id=sys.argv[2]


    rospy.init_node('vel_from_tf', anonymous=False)

    tl = tf.TransformListener()
    while not rospy.is_shutdown():

        try:


            pos,quat = tl.lookupTransform(marker_frame_id, map_frame_id, rospy.Time())
            tf_tools.save_tf(pos, quat,marker_frame_id,map_frame_id)
            break




        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('no tf found')

        rospy.Rate(10).sleep()
