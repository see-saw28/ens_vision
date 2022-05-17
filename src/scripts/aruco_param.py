#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from ens_vision.cfg import ArucoConfig

def callback(config, level):
            
            rospy.loginfo("""Reconfigure Request: {adaptiveThreshConstant}, {minMarkerPerimeterRate},{polygonalApproxAccuracyRate}, {perspectiveRemovePixelPerCell}, {maxErroneousBitsInBorderRate}""".format(**config))
            
            return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(ArucoConfig, callback)
    rospy.spin()
