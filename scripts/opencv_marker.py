#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 11 18:10:13 2022

@author: paul
"""
#!/usr/bin/env python

# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import pyrealsense2 as rs
import numpy as np
import os
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_5X5_50",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
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
pipeline.start(config)


draw=True

# loop over the frames from the video stream
while True:
    
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
    
    	# verify *at least* one ArUco marker was detected
        if len(corners) > 0:
    		# flatten the ArUco IDs list
            ids = ids.flatten()
    
    		# loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
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
                 
            mtx =np.array([[613.9803466796875, 0.0, 328.2522277832031], [0.0, 614.0032958984375, 234.93385314941406], [0.0, 0.0, 1.0]])
            
            
            dist = np.array( [0.0, 0.0, 0.0, -0.0, 0.0] )
            #POSE ESTIMATION
            rvec, tvec ,_ = cv2.aruco.estimatePoseSingleMarkers(np.array(corners), 0.05, mtx, dist)
            
            if draw:
                cv2.putText(frame, str(tvec[0,0,2])+"mm",(topLeft[0], bottomLeft[1] + 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                
                for i in range(0, ids.size):
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
    
    key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

# do a bit of cleanup
cv2.destroyAllWindows()
pipeline.stop()

# np.save('/path_to_images/calibration', [mtx, dist, rvecs, tvecs])

# calibration_data = np.load('path_to_images/calibration.npy')
# mtx = calibration_data[0]
# dist = calibration_data[1]

