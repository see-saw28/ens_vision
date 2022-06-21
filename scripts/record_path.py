#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 21 14:00:51 2022

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
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_matrix, quaternion_from_euler,euler_from_quaternion
import pickle
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path


frame_0='marker_0'
frame_1='marker_1'
frame_2='marker_2'

map_frame_id='map'
map_name = 'test_map'

number_of_points = 6

def normalize_angle(angle):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def calc_error(x, y, yaw, course_x, course_y, course_yaw):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    

    # Search nearest point index
    dx = [x - icx for icx in course_x]
    dy = [y - icy for icy in course_y]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    error_front_axle = min(d)
    target_idx = d.index(error_front_axle)
    # print(target_idx)

    error_yaw = normalize_angle(course_yaw[target_idx] - yaw)
        
    
    return error_front_axle, error_yaw

def path_callback(data):
    global course_x
    global course_y
    global course_yaw

    
    path_x = []
    path_y = []
    path_yaw = []
    
    for i, pose in enumerate(data.poses):
        path_x.append(data.poses[i].pose.position.x)
        path_y.append(data.poses[i].pose.position.y)
        orientation_list = [data.poses[i].pose.orientation.x, data.poses[i].pose.orientation.y, data.poses[i].pose.orientation.z, data.poses[i].pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        path_yaw.append(yaw)
    
    course_x = path_x
    course_y = path_y
    course_yaw = path_yaw
    

def callback(msg):
    global i
    global msg_path
    
    
    x0, y0, z0 = tl.lookupTransform('map', 'marker_0', rospy.Time())[0]
    x1, y1, z1 = tl.lookupTransform('map', 'marker_1', rospy.Time())[0]
    
    yaw = np.arctan2(y1 - y0, x1 - x0)
    
    error_front_axle, error_yaw = calc_error(x0, y0, yaw ,course_x, course_y, course_yaw)
    
    pub_error_pos.publish(Float32(error_front_axle))
    pub_error_yaw.publish(Float32(error_yaw))
    
    quat = quaternion_from_euler(0,0,yaw)
   
    
     
    
   
       
            
    pose = PoseStamped()
    
    pose.header.frame_id = "map"
    pose.header.seq = i
    
    pose.pose.position.x = x0
    pose.pose.position.y = y0
    
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    
    msg_path.poses.append(pose)
    
    pub.publish(msg_path)
    i+=1
    
    

if __name__ == '__main__':    
    
    rospy.init_node('record_path')
    tl = tf.TransformListener()
    pub = rospy.Publisher('current_path', Path, queue_size=10)
    pub_error_pos = rospy.Publisher('crosstrack_error', Float32, queue_size=10)
    pub_error_yaw = rospy.Publisher('yaw_error', Float32, queue_size=10)
    rospy.Subscriber('ref_path', Path, path_callback)
    msg_path = Path()
    msg_path.header.frame_id = 'map'
    i = 0
    
    
        
    
    
    try :
       rospy.Subscriber("aruco", Image, callback)
       
       rospy.spin()
            
            
        
        
    except rospy.ROSInterruptException:
           
            pass
    
    # R,T,df=svd_transform(si, di)


