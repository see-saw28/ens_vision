#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 13 15:20:41 2022

@author: student
"""
# -*- coding: utf-8 -*-
"""
Created on Thu May 19 22:50:21 2022

https://hal.inria.fr/inria-00073710/document

@author: paulg
"""
import numpy as np


di=np.array([[11.157,-0.089357,0.80816],[13.368, 1.6957, 2.7892],[12.681,2.2539,5.0729]])

# si=np.array([[1,3,2],[1, 1, 3],[1,1,1]])
si=np.array([[1,0,1],[3, 2, 3],[2,3,5]])


# di=np.array([[10,0,1],[12, 2, 1],[11,3,1]])
# d=di.reshape((3,3,1))
# # si=np.array([[1,3,2],[1, 1, 3],[1,1,1]])
# si=np.array([[1,0,1],[3, 2, 1],[2,3,1]])
# s=si.reshape((3,3,1))
def svd_transform(si,di):
    s=si.reshape((len(si),3,1))
    d=di.reshape((len(di),3,1))
    s_b=np.mean(s,axis=0)
    d_b=np.mean(d,axis=0)
    
    d_c=d-d_b
    s_c=s-s_b
    
    H=np.zeros((3,3))
    for i in range(len(s)):
        H+=np.dot(s_c[i],np.transpose(d_c[i]))
        
    # H=H/len(s)
    
    # H=np.dot(s_c.reshape((3,3)),np.transpose(d_c.reshape(3,3)))
    
    U,S,V = np.linalg.svd(H)
    
    V=np.transpose(V)
    
    
    R=np.dot(V,np.transpose(U))
    # print(R)
    if(np.linalg.det(R)<0):
        # print(V)
        V_p=np.copy(V)
        V_p[:,2]=-V[:,2]
        # print(V_p)
        R=np.dot(V_p,np.transpose(U))
    
    
    T=d_b-np.dot(R,s_b)
    
    df=np.transpose(np.dot(R,np.transpose(si))+T)
    return (R,T,df)


# import matplotlib.pyplot as plt
# import yaml

# a_yaml_file = open("map1.yaml")

# parsed_yaml_file = yaml.load(a_yaml_file, Loader=yaml.FullLoader) #dico

# pgm_name = parsed_yaml_file['image']
# resolution = parsed_yaml_file['resolution']
# xo, yo,_ = parsed_yaml_file['origin']


# with open('map1.pgm', 'rb') as pgmf:
#     im = plt.imread(pgmf)


import rospy
import tf
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_from_matrix
import pickle


frame_0='marker_0'
frame_1='marker_1'
frame_2='marker_2'

map_frame_id='map'
map_name = 'test_map'

number_of_points = 6


# R,T,df=svd_transform(si, di)
# rotation_matrix = np.array([[0, 0, 0, 0],
#                            [0, 0, 0, 0],
#                            [0, 0, 0, 0],
#                            [0, 0, 0, 1]],
#                            dtype=float)
# rotation_matrix[:3, :3] = R
# quat = quaternion_from_matrix(rotation_matrix)
# print(quat)

def callback(msg):
    global i
    x=msg.point.x
    y=msg.point.y
    z=0
    si[i%number_of_points]=[x,y,z]
    print(si)
    di[i%number_of_points] = tl.lookupTransform('camera', f'marker_{i}', rospy.Time())[0]
    print(di)
    if i%number_of_points==(number_of_points-1):
     
        
        R,T,df=svd_transform(si, di)
        
        pos = T
        rotation_matrix = np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 1]],
                                   dtype=float)
        rotation_matrix[:3, :3] = R
        quat = quaternion_from_matrix(rotation_matrix)
        print(R)
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(30)
        
        import os
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
        
        import rospkg
        rospack = rospkg.RosPack()
        
        
             
        filename=check_file(rospack.get_path('ens_voiture_autonome')+f'/tf/{map_name}.pckl')
        
        f = open(filename, 'wb')
        pickle.dump([pos,quat,'map','camera'], f)
        f.close()
        
        print('saved mcp path :', filename)
        
        
        while not rospy.is_shutdown():

            
            br.sendTransform(pos,quat,rospy.Time.now(),map_frame_id,'camera')
            
            
            rate.sleep()
        
        
    i+=1
    
    
if __name__ == '__main__':    
    i=0
    di=np.zeros((number_of_points,3))
    si=np.zeros((number_of_points,3))
    rospy.init_node('svd')
    tl = tf.TransformListener()
    
    try:
        rospy.Subscriber("clicked_point", PointStamped, callback)
        
        rospy.spin()
        
        
    except rospy.ROSInterruptException:
           
            pass
    
    # R,T,df=svd_transform(si, di)


