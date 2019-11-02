#!/usr/bin/env python
#coding=utf-8
import rospy
import sys, select, termios, tty
from drive.msg import Move


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('send_move_cmd')
    pub=rospy.Publisher('/control/ismove',Move,queue_size=1)
    move=Move()
    rate=rospy.Rate(10)
    rospy.loginfo('v启动  空格停止')
    while(1):
        key=getKey()
        if key=='v':
            move.state=True
            rospy.loginfo('车启动！')
            pub.publish(move)
        elif  key==' ':
            move.state=False
            rospy.loginfo('车停止！')
            pub.publish(move)
        if (key == '\x03'):
            break
        rate.sleep()