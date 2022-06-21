#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 09:12:47 2022

@author: student
"""
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
    
    print('0->1',np.linalg.norm(tl.lookupTransform('marker_0', 'marker_1', rospy.Time())[0]))
    print('1->2',np.linalg.norm(tl.lookupTransform('marker_1', 'marker_2', rospy.Time())[0]))
    print('2->3',np.linalg.norm(tl.lookupTransform('marker_2', 'marker_3', rospy.Time())[0]))
    print('3->4',np.linalg.norm(tl.lookupTransform('marker_3', 'marker_4', rospy.Time())[0]))
    print('4->5',np.linalg.norm(tl.lookupTransform('marker_4', 'marker_5', rospy.Time())[0]))
    
    
    
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


