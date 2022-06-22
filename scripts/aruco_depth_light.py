#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 14:59:20 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 21 10:24:07 2022

@author: student
"""
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
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
import tf
from dynamic_reconfigure.server import Server
from ens_vision.cfg import ArucoConfig




# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys
import pyrealsense2 as rs
import os
from cv_bridge import CvBridge

ir1_image = []
depth_image = []

def file_constante(liste,element,longueur):
    liste.append(element)
    if len(liste)>longueur:
        liste.pop(0)
    return liste
    
        
def callback(config, level):
            
            rospy.loginfo("""Reconfigure Request: {adaptiveThreshConstant}, {minMarkerPerimeterRate},{polygonalApproxAccuracyRate}, {perspectiveRemovePixelPerCell}, {maxErroneousBitsInBorderRate}""".format(**config))
            
            return config

def img_callback(data):
    global ir1_image
    bridge = CvBridge()
    ir1_image = bridge.imgmsg_to_cv2(data)
    
def depth_callback(data):
    global depth_image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data)
    
def param_callback(data):
    global dist
    global mtx
    
    dist = np.array(data.D)
    
    
    P = np.array(data.P)
    mtx = np.array([P[0:3],P[4:7],P[8:11]])

    #mtx = np.array([[intr.fx, 0, intr.ppx],[0, intr.fy, intr.ppx],[0, 0, 1]])   

    

def aruco():
  
    global bruit_aruco
    global bruit_realsense
    br = tf.TransformBroadcaster()
    rospy.init_node('aruco_frame_publisher')
    rospy.Subscriber('/camera/infra1/image_rect_raw', Image, img_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    
    rospy.Subscriber('/camera/infra1/camera_info', CameraInfo, param_callback)
    srv = Server(ArucoConfig, callback)
    pubAruco = rospy.Publisher('aruco', Image, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    
    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
    while not rospy.is_shutdown():
        if ir1_image != [] :
            arucoParams.adaptiveThreshWinSizeMin=rospy.get_param('aruco/adaptiveThreshWinSizeMin')
            arucoParams.adaptiveThreshWinSizeMax=rospy.get_param('aruco/adaptiveThreshWinSizeMax')
            arucoParams.adaptiveThreshWinSizeStep=rospy.get_param('aruco/adaptiveThreshWinSizeStep')
            arucoParams.adaptiveThreshConstant=rospy.get_param('aruco/adaptiveThreshConstant')
            arucoParams.minMarkerPerimeterRate=rospy.get_param('aruco/minMarkerPerimeterRate')
            arucoParams.polygonalApproxAccuracyRate=rospy.get_param('aruco/polygonalApproxAccuracyRate')
            arucoParams.cornerRefinementMethod=rospy.get_param('aruco/cornerRefinementMethod')
            arucoParams.perspectiveRemovePixelPerCell=rospy.get_param('aruco/perspectiveRemovePixelPerCell')
            arucoParams.maxErroneousBitsInBorderRate=rospy.get_param('aruco/maxErroneousBitsInBorderRate')
            arucoParams.errorCorrectionRate=rospy.get_param('aruco/errorCorrectionRate')
            
            
       
            
            # Convert to RGB image for axis drawing
            gray = cv2.cvtColor(ir1_image, cv2.COLOR_GRAY2RGB)
           
            
        	# detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(ir1_image,arucoDict, parameters=arucoParams)
            
            
            #mtx =np.array([[613.9803466796875, 0.0, 328.2522277832031], [0.0, 614.0032958984375, 234.93385314941406], [0.0, 0.0, 1.0]])
           
            #dist = np.array( [0.0, 0.0, 0.0, -0.0, 0.0] )
            
           
            
            
        
        	# verify *at least* one ArUco marker was detected
            if len(corners) > 0:
        		# flatten the ArUco IDs list
                ids = ids.flatten()
        
                # loop over the detected ArUCo corners
                for i in range(0, ids.size):
                     markerCorner, markerID = corners[i], ids[i]
                     # extract the marker corners (which are always returned
                    # in top-left, top-right, bottom-right, and bottom-left
			        # order)
                     corner = markerCorner.reshape((4, 2))
                     
                     (topLeft, topRight, bottomRight, bottomLeft) = corner
         
                     # convert each of the (x, y)-coordinate pairs to integers
                     topRight = (int(topRight[0]), int(topRight[1]))
                     bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                     bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                     topLeft = (int(topLeft[0]), int(topLeft[1]))
                     
                     cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                     cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                     
                     
                     if markerID<2:
                         size=0.8
                     else:
                         pass
                     
                     
                     
                     
                      
                           
                     
                    
                    # print(f'marker {markerID} ',f'{tvec[0,0,2]:.3f}+/-{np.std(bruit_aruco[markerID]):.5f}'+"m", f'{depth_image[cY,cX]/1000}+/-{np.std(bruit_realsense[markerID]):.5f}m')
                    
                     (topLeft1, topRight1, bottomRight1, bottomLeft1) = corner
                     cX1 = (topLeft1[0] + bottomRight1[0]) / 2.0
                     cY1 = (topLeft1[1] + bottomRight1[1]) / 2.0
                    
                     z = depth_image[int(cY1),int(cX1)]/1000
                    
                     #pinhole model
                     x = (cX1 - mtx[0,2]) * z / mtx[0,0]
                     y = (cY1 - mtx[1,2]) * z / mtx[1,1]
                
                     
                          
                     
                     
                    
                     br.sendTransform((x,y,z),
                                     (0,0,0,1),
                                     rospy.Time.now(),f"marker_{ids[i]}","camera")
                         
         
            
                     
                        
                                        
        
    	
    	
            
            
            rate.sleep() 
            
          
            

if __name__ == '__main__':
    try:
    
        # Configure aruco detection
        # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoDict = cv2.aruco.Dictionary_create(6, 3)
        arucoParams = cv2.aruco.DetectorParameters_create()
        
	
        bruit_aruco = [[],[],[],[],[],[],[]]
        bruit_realsense = [[],[],[],[],[],[],[]]
        
        draw=False
        draw_cv2=False
        debug=False
        print('start')
        # print("press 'q' to close")
        aruco()
    except rospy.ROSInterruptException:
        # do a bit of cleanup
        cv2.destroyAllWindows()
        pass
