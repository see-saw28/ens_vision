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
from sensor_msgs.msg import Image
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
    ir1_image = bridge.imgmsg_to_cv2(data, encoding="passthrough")
    print(type(ir1_image))

def aruco():
  
    global bruit_aruco
    global bruit_realsense
    br = tf.TransformBroadcaster()
    rospy.init_node('aruco_frame_publisher')
    pubIR = rospy.Subscriber('/camera/infra1/image_rect_raw', Image, img_callback)
    srv = Server(ArucoConfig, callback)
    pubAruco = rospy.Publisher('aruco/IRLeft', Image, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    
    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
    while not rospy.is_shutdown():
        try :
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
            
            # Get intrinsics params of the camera
            profile = cfg.get_stream(rs.stream.infrared) 
            intr = profile.as_video_stream_profile().get_intrinsics()
            
            mtx = np.array([[intr.fx, 0, intr.ppx],[0, intr.fy, intr.ppx],[0, 0, 1]])   

            dist = np.array( intr.coeffs )
            
            if debug:
            
                for rej in rejected:
                     
                     # extract the marker corners (which are always returned
                    # in top-left, top-right, bottom-right, and bottom-left
           # order)
                     corner = rej.reshape((4, 2))
                     
                     (topLeft, topRight, bottomRight, bottomLeft) = corner
         
                     # convert each of the (x, y)-coordinate pairs to integers
                     topRight = (int(topRight[0]), int(topRight[1]))
                     bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                     bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                     topLeft = (int(topLeft[0]), int(topLeft[1]))
                     
                     cv2.line(gray, topLeft, topRight, (0, 255, 255), 2)
                     cv2.line(gray, topRight, bottomRight, (0, 255, 255), 2)
                     cv2.line(gray, bottomRight, bottomLeft, (0, 255, 255), 2)
                     cv2.line(gray, bottomLeft, topLeft, (0, 255, 255), 2)
        
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
                     
                     
                     if markerID==0:
                         size=0.08
                     else:
                         size=0.08
                     
                     # POSE ESTIMATION
                     rvec, tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(np.array(markerCorner), size, mtx, dist)
                                        
                     x,y,z=tvec[0][0]
                     a,b,c=rvec[0][0]
                     
                     
                          
                     
                     #rvec angle rodrigues but ROS needs quaternion
                     # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
                     rotation_matrix = np.array([[0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 1]],
                                                dtype=float)
                     rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec[0])
                    
                    # convert the matrix to a quaternion
                     quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
                    
                     br.sendTransform((x,y,z),
                                     quaternion,
                                     rospy.Time.now(),f"marker_{ids[i]}","camera")
                         
         
            
                     if draw:
             			# draw the bounding box of the ArUCo detection
                         cv2.line(gray, topLeft, topRight, (0, 255, 0), 2)
                         cv2.line(gray, topRight, bottomRight, (0, 255, 0), 2)
                         cv2.line(gray, bottomRight, bottomLeft, (0, 255, 0), 2)
                         cv2.line(gray, bottomLeft, topLeft, (0, 255, 0), 2)
                         
                         
                         
                         
             
             			# compute and draw the center (x, y)-coordinates of the
             			# ArUco marker
                         
                         cv2.circle(gray, (cX, cY), 4, (0, 0, 255), -1)
             
             			# draw the ArUco marker ID on the frame
                         cv2.putText(gray, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                         # draw the ArUco marker ID on the frame
                         

                        
                         cv2.putText(gray, f'{tvec[0,0,2]:.3f}'+"m",(topLeft[0], topLeft[1] + 35),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                         
                         print(f'{tvec[0,0,2]:.3f}'+"m")
                             
                         
                         
                   
                         # draw axis for the aruco markers
                         cv2.aruco.drawAxis(gray, mtx, dist, rvec[0], tvec[0], 0.1)
                        
                                        
        except Exception as e: 
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            print(e)
            break
    	
    	
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(gray, encoding="passthrough")
        pubAruco.publish(image_message)
        if draw_cv2:
        	# show the output frame
            cv2.imshow("IR Left", gray)
            
        
        key = cv2.waitKey(1) & 0xFF
    
    	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
            
        
        rate.sleep() 
            
            
            

if __name__ == '__main__':
    try:
    
        # Configure aruco detection
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        
	
        bruit_aruco = []
        bruit_realsense = []
        
        draw=True
        draw_cv2=True
        debug=True
        print('start')
        # print("press 'q' to close")
        aruco()
    except rospy.ROSInterruptException:
        # do a bit of cleanup
        cv2.destroyAllWindows()
        pass
