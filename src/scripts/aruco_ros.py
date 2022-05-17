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
import tf




# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import cv2
import sys
import pyrealsense2 as rs
import os




def aruco():
  
    br = tf.TransformBroadcaster()
    rospy.init_node('aruco_frame_publisher')
    rate = rospy.Rate(30) # 30hz
    
    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
    while not rospy.is_shutdown():
        try :
        	# grab the frame from the threaded video stream and resize it
        	# Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            
        
            # Create alignment primitive with color as its target stream:
            align = rs.align(rs.stream.color)
            frames = align.process(frames)
            
            # Update color and depth frames:
            aligned_depth_frame = frames.get_depth_frame()
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            frame = np.asanyarray(color_frame.get_data())
            
        	# detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,arucoDict, parameters=arucoParams)
            
            
            #mtx =np.array([[613.9803466796875, 0.0, 328.2522277832031], [0.0, 614.0032958984375, 234.93385314941406], [0.0, 0.0, 1.0]])
           
            #dist = np.array( [0.0, 0.0, 0.0, -0.0, 0.0] )
            
            # Get intrinsics params of the camera
            profile = cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
            intr = profile.as_video_stream_profile().get_intrinsics()
            
            mtx = np.array([[intr.fx, 0, intr.ppx],[0, intr.fy, intr.ppx],[0, 0, 1]])   

            dist = np.array( intr.coeffs )
            
            
            
            #POSE ESTIMATION
            rvec, tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(np.array(corners), 0.08, mtx, dist)
        
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
                     
                     d=depth_image[cY,cX]
                     
                                  
                     x,y,z=tvec[i][0]
                     a,b,c=rvec[i][0]
                     
                     # Use opencv distance if depth camera return 0
                     if (d==0):
                          d=z*1000
                          
                     
                     #rvec angle rodrigues et ROS quaternion
                     # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
                     rotation_matrix = np.array([[0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 1]],
                                                dtype=float)
                     rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec[i])
                    
                    # convert the matrix to a quaternion
                     quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
                    
                     br.sendTransform((x,y,d/1000),
                                     quaternion,
                                     rospy.Time.now(),f"marker_{ids[i]}","camera")
                         
         
            
                     if draw:
             			# draw the bounding box of the ArUCo detection
                         cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                         cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                         cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                         cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
             
             			# compute and draw the center (x, y)-coordinates of the
             			# ArUco marker
                         
                         cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
             
             			# draw the ArUco marker ID on the frame
                         cv2.putText(frame, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                         # draw the ArUco marker ID on the frame
                         cv2.putText(frame, str(depth_image[cY,cX])+"mm",(topLeft[0]+25, topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                     
               
                
                
                         cv2.putText(frame, f'{tvec[i,0,2]:.3f}'+"m",(topLeft[0], bottomLeft[1] + 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    
                   
                         # draw axis for the aruco markers
                         cv2.aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
                        
                                        
        except Exception as e: 
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            print(e)
            break
    
        if draw:
        	# show the output frame
            cv2.imshow("Frame", frame)
            cv2.imshow("Depth Frame", depth_image)
        
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
        
	# Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

	# Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False

        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start streaming
        cfg = pipeline.start(config)
        
        
        draw=True
        print('start')
        print("press 'q' to close")
        aruco()
    except rospy.ROSInterruptException:
        # do a bit of cleanup
        cv2.destroyAllWindows()
        pipeline.stop()
        pass
