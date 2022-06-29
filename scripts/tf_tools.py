#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 18 11:42:09 2022

@author: student
"""



import rospy
import numpy as np
import tf
import sys
import os

import pickle
import rospkg


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

def save_tf(pos, quat, child_frame, parent_frame, name='tf'):
    rospack = rospkg.RosPack()

    filename=check_file(rospack.get_path('ens_vision')+f'/tf/{name}.pckl')

    f = open(filename, 'wb')
    pickle.dump([pos,quat,child_frame,parent_frame], f)
    f.close()

    rospy.loginfo('saved tf :'+ filename)

    return filename

def load_tf(name, absolute_path=False, rate=100):



    rospy.init_node(f'load_tf_{name}', anonymous=False)
    RATE = rospy.Rate(rate)

    br = tf.TransformBroadcaster()



    import rospkg
    rospack = rospkg.RosPack()
    if absolute_path :
        f = open(name, 'rb')
    else :
        f = open(rospack.get_path('ens_vision')+f'/tf/{name}.pckl', 'rb')
    pos,quat,child_frame, parent_frame = pickle.load(f)
    print(pos,quat)
    f.close()



    while not rospy.is_shutdown():


        br.sendTransform(pos,quat,rospy.Time.now(),child_frame, parent_frame)
        # print(map_frame_id)

        RATE.sleep()



if __name__ == '__main__':
    try:
        if (len(sys.argv)>1):
            name=sys.argv[1]
        else :
            name = 'map5'

        load_tf(name)
    except rospy.ROSInterruptException:
        pass




