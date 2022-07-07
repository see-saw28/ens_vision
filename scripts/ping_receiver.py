#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 09:54:27 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 09:48:23 2022

@author: student
"""
import rospy
from geometry_msgs.msg import PointStamped


def callback(data):
    pub.publish(data)
    rospy.loginfo('Ping received')



if __name__ == '__main__':
    i=0
    rospy.init_node('ping_receiver')
    pub = rospy.Publisher('ping_down', PointStamped, queue_size=1)
    rospy.Subscriber("ping_up", PointStamped, callback)


    try:


        rospy.spin()


    except rospy.ROSInterruptException:

            pass

