#!/usr/bin/env python
#coding=utf-8
import rospy
import math
from drive.msg import dat

y = dat()

def callback(x):

    y.a = x.a + 1
    print x.a
def talker():

    rospy.init_node('talker')
    rospy.Subscriber('chatter2',dat,callback)
    pub = rospy.Publisher('chatter1',dat,queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(y)
        rospy.sleep(1.0)
    rospy.spin()
if __name__ == '__main__':
    try:talker()
    except rospy.ROSInterruptException: pass