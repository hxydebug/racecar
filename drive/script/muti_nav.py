#!/usr/bin/env python
#coding=utf-8
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from drive.msg import muxMsg


class MultiGoal:
    def __init__(self):
        rospy.Subscriber('move_base_simple/isgetGoal', muxMsg, self.muxMsg_cb)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.recvGoal)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)

        self.mg = muxMsg()
        self.goals = []

        self.count = int(input('请输入导航点数'))
        self.count_get = 0

        i = 0
        pre_count = self.count_get-1
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if len(self.goals) < self.count:
                continue
            else:
                if (self.count_get-pre_count)==1 and i<self.count:   
                    now = rospy.get_rostime()
                    self.goals[i].header.stamp = now
                    self.goal_pub.publish(self.goals[i])
                    rospy.loginfo('发送接收到的第%i点',i+1)
                    self.mg.m = False
                    i += 1
                    pre_count = self.count_get

                    if i == len(self.goals):
                        rospy.loginfo('发送完毕')       
            rate.sleep()


    def recvGoal(self, msg):
        goal_ = msg
        if len(self.goals)<self.count:
            self.goals.append(goal_)
            rospy.loginfo('接收到第%i点',len(self.goals))
            # print self.goals[-1]
        
    def muxMsg_cb(self,mux_msg):
        self.mg = mux_msg
        if self.mg.m:
            self.count_get+=1
            print self.count_get

        
if __name__=="__main__":
    rospy.init_node('MultiGoal')
    node = MultiGoal()
    rospy.spin()