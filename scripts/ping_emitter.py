#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 09:48:23 2022

@author: student
"""
import rospy
from geometry_msgs.msg import PointStamped
import time

def callback(data):
    # global sent_time
    sent_time = data.header.stamp.to_sec()
    print(rospy.Time.now().to_sec()-sent_time)

    # time.sleep(0.1)

    # point= PointStamped()
    # point.header.stamp = rospy.Time.now()
    # pub.publish(point)

    # sent_time = point.header.stamp.to_sec()


if __name__ == '__main__':
    i=0
    rospy.init_node('ping_emitter')
    pub = rospy.Publisher('ping_up', PointStamped, queue_size=1)
    rospy.Subscriber("ping_down", PointStamped, callback)


    rate = rospy.Rate(1)

    # sent_time = point.header.stamp.to_sec()
    try:
        while not rospy.is_shutdown():
            point= PointStamped()
            point.header.stamp = rospy.Time.now()
            pub.publish(point)

            # print(point)
            rospy.loginfo('Ping sent')
            rate.sleep()
            # rospy.spin()


    except rospy.ROSInterruptException:

            pass

