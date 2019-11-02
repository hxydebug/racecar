#!/usr/bin/env python
#coding=utf-8
import rospy
import math
import tf

from sensor_msgs.msg import LaserScan



def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I d %s', len(data.ranges))
    print data.angle_min,data.angle_max,data.angle_increment,data.range_min,data.range_max
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber('scan', LaserScan, callback)
    rospy.Subscriber('scan_kinect', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
