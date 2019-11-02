#!/usr/bin/env python
#coding=utf-8
import rospy
import ser
import math
import tf
import time
from drive.msg import Move,muxMsg
from geometry_msgs.msg import Twist,Quaternion
from nav_msgs.msg import Odometry


class driver:
    def __init__(self, odom=True):
        self.use_odom = odom
        try:
            port_siglechip = rospy.get_param('~port_siglechip')
            self.com=ser.Serial(port_siglechip)
        
        except Exception,e:
            print e
        
        rospy.Subscriber( "/control/cmd_vel", Twist , self.cmd_callback)
        rospy.Subscriber( "/control/ismove" , Move , self.move_callback)

        self.pub=rospy.Publisher('/control/odom_encoder',Odometry,queue_size=20)
        self.mux_pub=rospy.Publisher('/control/close_loop_msg',muxMsg,queue_size=10)

        self.ismove=False
        self.mux_msg=muxMsg()

        self.resolve=0.20000/1000   # 单位编码数走过实际距离 米制
        self.track=0.20     # 轮子之间距离
        self.wheelbase=0.335 #车前后轴距离

        self.rever_kp = 30 ; self.rever_kd = 10 ; self.p = self.d = self.error = self.pre_error = 0  #向后反转刹车pd参数
        self.car_speed = 0
        self.odometry=Odometry()

        self.steer = 0


        self.car_speed = 0
        self.odometry=Odometry()
        self.turn_tha_old=0

        self.rev_pd = {
          0:( 30 , 18 ),
        0.5:( 30 , 18 ),
          1:( 50 , 18 ),
        1.5:( 40 , 10 ),
          2:( 30 , 15 ),
        2.5:( 28 , 10 ),
          3:( 30 , 25 ),
        3.5:( 30 , 25 ),
          4:( 30 , 30 ),
        4.5:( 30 , 30 ),
            }
        self.rev_kp = self.rev_kd = self.rev_p = self.rev_d = self.rev_error = self.rev_pre_error = 0

        self.handle_siglechip()

    def pd_rever(self):
        if self.car_speed>4:
            v = 4.5
        elif self.car_speed>3.5:
            v = 4
        elif self.car_speed>3:
            v = 3.5 
        elif self.car_speed>2.5:
            v = 3
        elif self.car_speed>2:
            v = 2.5
        elif self.car_speed>1.5:
            v = 2
        elif  self.car_speed>1:
            v = 1.5
        elif  self.car_speed>0.5:
            v = 1
        elif  self.car_speed>0:
            v = 0.5
        else:
            v = 0
    
        self.rev_kp = self.rev_pd[v][0]
        self.rev_kd = self.rev_pd[0][1]

        self.rev_error = self.car_speed

        self.rev_p = self.rev_error
        self.rev_d = self.rev_error - self.rev_pre_error
        self.rev_pre_error = self.rev_error

        pid = self.rev_kp*self.rev_p + self.rev_kd*self.rev_d
        return pid

    def move_callback(self,msg):
        self.ismove=msg.state
        if  self.ismove:
            rospy.loginfo('车可以移动!')  
        else: 
            self.com.send_pwm(1500,1500)
            rospy.loginfo('车不可以移动!') 
    def cmd_callback(self,twist):
        # if twist.angular.z<90:
        #     pwm_angle = 0.100561*twist.angular.z*twist.angular.z*twist.angular.z-23.7661*twist.angular.z*twist.angular.z+1904.0912*twist.angular.z-50689.22375457
        # elif twist.angular.z>90: 
        #     pwm_angle = 0.091081*twist.angular.z*twist.angular.z*twist.angular.z-26.76655*twist.angular.z*twist.angular.z+2647.521*twist.angular.z-86351.11
        # else:
        #     pwm_angle = 1500

        pwm_angle = (twist.angular.z - 90)*500/35 + 1500
        pwm_angle = int(pwm_angle)
        pwm_motor=int(twist.linear.x)


        if pwm_angle>2400:
            pwm_angle=2400
        elif pwm_angle<600:
            pwm_angle=600

        #pwm限制
        if pwm_motor>2000:
            pwm_motor = 2000
        elif pwm_motor<1300:
            pwm_motor = 1300


        if self.ismove:
            if (pwm_angle==1500 and pwm_motor==1500) or twist.linear.z == 1:  #pursuit停车 通过比例反转刹车
                if self.car_speed>0.09:
                    pwm_motor = int(1500-self.pd_rever())
                    rospy.loginfo('x=%f    z=%f    车可以移动 到达目标刹车！'%(pwm_motor,pwm_angle))   
            rospy.loginfo('x=%f    z=%f    车可以移动！'%(pwm_motor,pwm_angle))   
            self.com.send_pwm(pwm_motor,pwm_angle)

        else: 
            if self.car_speed>0.4:
                #刹车
                pwm_motor = int(1500-self.pd_rever())
                rospy.loginfo('x=%f    z=%f    1车停止!'%(pwm_motor,pwm_angle))
                self.com.send_pwm(pwm_motor,pwm_angle)
            else:
                #停车仍然保持车轮转向
                if pwm_angle>1500:
                    pwm_angle = int(pwm_angle*1.2)
                elif pwm_angle<1500:
                     pwm_angle = int(pwm_angle*0.8)

                if pwm_angle>2500:
                    pwm_angle = 2500
                elif pwm_angle<500:
                    pwm_angle=500
                
                rospy.loginfo("x=1500   z=%f  2车停止!",pwm_angle)
                self.com.send_pwm(1500,pwm_angle)


    def handle_siglechip(self):
        mycom=self.com.search_com()
        for each_com in mycom:
            print each_com

        if self.com.clean_odom():
            print '清除encoder成功'
        else:
            print '清除encoder失败'

        
        now=then=rospy.get_rostime()
        pre_right_encoder=pre_left_encoder=0
        ds = th=x=y=0.0


        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                get_odom_success = self.com.get_odom()
            except Exception:
                time.sleep(.1)
                pass
            if self.use_odom and get_odom_success:
                # print self.com.odom_left,self.com.odom_right

                dt=now-then 
                dt=dt.secs+dt.nsecs*1e-9  #单位秒
                # print dt
                then=now
                now=rospy.get_rostime()

                # # 单位米制 计算左右轮转动距离 编码器改变数*分辨率
                ls=(self.com.odom_left-pre_left_encoder)*self.resolve
                rs=(self.com.odom_right-pre_right_encoder)*self.resolve
                # print ls,rs

                pre_left_encoder=self.com.odom_left
                pre_right_encoder=self.com.odom_right

                # # # base_link参数计算
                s=(ls+rs)/2         #中心平均位移
                angle=(rs-ls)/self.track    #转过的角度
                if dt==0.0:
                    ds=0
                    dth=0
                else:
                    ds=s/dt             #速度
                    dth=angle/dt        #角速度
                    # rospy.loginfo('s=%f    th=%f    ds=%f   dth=%f'%(s,angle,ds,dth))

                self.car_speed = ds

                dx=math.cos(angle)*s          #在base_link x轴的位移
                dy=math.sin(angle)*s          #在base_link y轴的位移
                x+=(math.cos(th)*dx-math.sin(th)*dy) #odom x轴的位移
                y+=(math.sin(th)*dx+math.cos(th)*dy) #odom y轴的位移

                th+=angle
                # rospy.loginfo('th=%f x=%f y=%f'%(th,x,y))
                
                quaternion=Quaternion()
                quaternion.x=0.0
                quaternion.y = 0.0
                quaternion.z = math.sin(th / 2.0)
                quaternion.w = math.cos(th / 2.0)

                # # 发布tf sendTransform(self, translation, rotation, time, child, parent):
                # self.br.sendTransform( (x, y, 0) , (quaternion.x, quaternion.y, quaternion.z, quaternion.w) , rospy.Time.now() , "base_link" , "odom" )
                
                # 发布odom
                odom = Odometry()

                odom.header.stamp = now
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_footprint"
                
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0

                odom.pose.pose.orientation = quaternion

                odom.twist.twist.linear.x = ds
                odom.twist.twist.linear.y = 0
                odom.twist.twist.linear.z = 0

                odom.twist.twist.angular.x = 0
                odom.twist.twist.angular.y = 0
                odom.twist.twist.angular.z = dth

                self.pub.publish(odom)


            else:
                self.car_speed = 0
                print "error:无法获取编码器数据"
                
            self.mux_msg.a = self.car_speed
            self.mux_pub.publish(self.mux_msg)
            rate.sleep()


    def __del__(self):
        self.com.send_pwm(1500,1500)
        print 'drive节点'
if __name__=="__main__":
    rospy.init_node('drive')
    driver=driver()
    rospy.spin()
