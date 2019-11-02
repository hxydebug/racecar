#!/usr/bin/env python
#coding=utf-8
import rospy
import math
from drive.msg import dat

y = dat()

def callback(x):
    
    y.a = x.a*3.0
    print x.a

def listener():
    
    rospy.init_node('listener')
    rospy.Subscriber('chatter1',dat,callback)
    pub = rospy.Publisher('chatter2',dat,queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(y)
        rospy.sleep(1.0)

    rospy.spin()
if __name__ == '__main__':
    try:listener()
    except rospy.ROSInterruptException: pass

