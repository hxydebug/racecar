#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped,Twist,Point,Pose,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from drive.msg import muxMsg

class Pursuit:
    def __init__(self):
        self.points = Marker()
        self.line_strip = Marker()
        self.goal_circle = Marker()

        self.odom_goal_pos = Point()
        self.odom = Odometry()
        self.map_path = Path()
        self.mux_msg = muxMsg()
        self.mg= muxMsg()
        self.scan_date = LaserScan()

        self.L=rospy.get_param('~L',default=0.335)
        self.Vcmd=rospy.get_param('~Vcmd',default=1.5) 
        self.lfw=rospy.get_param('~lfw',default=0.13) 

        self.controller_freq=rospy.get_param('~controller_freq',default=20)
        self.base_angle=rospy.get_param('~baseAngle',90.0)
        self.speed_max=2
        self.speed_turn_max=1.5
        rospy.Subscriber('/scan',LaserScan,self.scan_cb) 
        # rospy.Subscriber('/odom',Odometry,self.odom_cb)
        rospy.Subscriber('/odometry/filtered',Odometry,self.odom_cb)
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan_odom',Path,self.path_cb) 
        rospy.Subscriber('/move_base_simple/goal_odom',PoseStamped,self.goal_cb) 
        rospy.Subscriber('/control/close_loop_msg',muxMsg,self.mux_cb)
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.amcl_pose_cb)

        self.marker_pub = rospy.Publisher('/control/car_path',Marker,queue_size=10)
        self.cmd_pub = rospy.Publisher('/control/cmd_vel',Twist,queue_size=2)
        self.goal_reached_pub = rospy.Publisher('/move_base_simple/isgetGoal',muxMsg,queue_size=10)

        self.amcl_pose = PoseWithCovarianceStamped()
        self.goalRadius = 0.2 #车距离目标点的距离半径
        self.Lfw  =  0.8
        # self.getL1Distance(self.Vcmd)  #前视距离和速度成正比 
        self.foundForwardPt = self.goal_received = self.goal_reached =False

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 1500
        self.cmd_vel.angular.z = self.base_angle

        # 前视角
        self.eta_pre = 0

        # p i d Lfw
        self.move_steer_pid = {
          1:( 4   , 0 , 1 , 1    ),
        1.5:( 4   , 0 , 1 , 1    ),
          2:( 4   , 0 , 1 , 1.1  ),
        2.5:( 3   , 0 , 1 , 1.2  ),
          3:( 3   , 0 , 1 , 1.25 ),
        3.5:( 3   , 0 , 1 , 1.3  ),
          4:( 3   , 0 , 1 , 1.35 ),
        4.5:( 3   , 0 , 1 , 1.4  ),
          5:( 3   , 0 , 1 , 1.45 )
           }


        #舵机控制及速度控制pid参数
        self.steer_kp= self.steer_ki =  self.steer_kd = self.steer_p = self.steer_i = self.steer_d = self.steer_error = self.steer_pre_error = 0
        self.steering_angle = 0  #应该输出的舵机角度

        # p i d pwm
        self.move_speed_pid = {
          1:( 30 , 0 , 10 , 1500),
        1.5:( 35 , 0 , 15 , 1500),
          2:( 38 , 0 , 20 , 1500),
        2.5:( 35 , 0 , 25 , 1500),
          3:( 40 , 0 , 30 , 1500),
        3.5:( 50 , 0 , 35 , 1500),
          4:( 55 , 0 , 38 , 1500),
        4.5:( 60 , 0 , 41 , 1500),
          5:( 65 , 0 , 45 , 1500)
           }

        #前进pid
        self.speed_kp= self.speed_ki = self.speed_kd = self.speed_p = self.speed_i = self.speed_d = self.speed_error = self.speed_pre_error = 0
        self.speed_v = self.Vcmd   #应该达到的速度


        rospy.loginfo("[param] base_angle: %f", self.base_angle)
        rospy.loginfo("[param] Vcmd: %f", self.Vcmd)
        rospy.loginfo("[param] Lfw: %f", self.Lfw)
        rospy.loginfo("[param] lfw: %f", self.lfw)
        self.init_marker()

        rate=rospy.Rate(self.controller_freq)
        while not rospy.is_shutdown():
            self.goalReaching()
            self.controlLoop()
            rate.sleep()


    def odom_cb(self,odom):#/odometry/filtered',Odometry
        self.odom=odom
    def amcl_pose_cb(self,pose):
        self.amcl_pose = pose

    def scan_cb(self,laser):#/scan
        self.scan_date = laser
        # print self.scan_date

    def path_cb(self,plan):
        self.map_path=plan  #frame_id=odom  /move_base/TebLocalPlannerROS/local_plan
    def mux_cb(self,mux_msg):
        self.mux_msg = mux_msg

    def goal_cb(self,odom_goal):#'/move_base_simple/goal',PoseStamped
        try:
            self.odom_goal_pos = odom_goal.pose.position
            self.goal_received = True
            self.goal_reached = False
            #在rviz中画出目标
            self.goal_circle.pose = odom_goal.pose
            self.marker_pub.publish(self.goal_circle)
        except Exception,e:
            print e
            rospy.loginfo("def goal_cb(self,goal) 异常!")

    def goalReaching(self):
        if self.goal_received:
            distance = self.getCar2GoalDist()
            if distance < self.Lfw:
                self.goal_reached = True
                self.goal_received = False
                print '到达目标'
                self.mg.m = True
                self.goal_reached_pub.publish(self.mg)

            else:
                self.mg.m = False
                self.goal_reached_pub.publish(self.mg)


    def controlLoop(self):
        # p i d Lfw
        self.move_steer_pid = {
          1:( 1    , 0 , 1 , 1    ),
        1.5:( 0.95 , 0 , 0.1 , 1    ),
          2:( 0.95    , 0 , 0.1 , 1.1  ),
        2.5:( 1.2    , 0 , 0.1 , 1.2  ),
          3:( 3    , 0 , 0.1 , 1.25 ),
        3.5:( 3    , 0 , 0.1 , 1.3  ),
          4:( 3    , 0 , 0.1 , 1.35 ),
        4.5:( 3    , 0 , 0.1 , 1.4  ),
          5:( 3    , 0 , 0.1 , 1.45 )
           }
        # p i d pwm
        self.move_speed_pid = {
          1:( 30 , 0 , 10 , 1560),
        1.5:( 35 , 0 , 15 , 1570),
          2:( 38 , 0 , 20 , 1585),
        2.5:( 35 , 0 , 25 , 1595),
          3:( 40 , 0 , 30 , 1605),
        3.5:( 50 , 0 , 35 , 1615),
          4:( 55 , 0 , 38 , 1625),
        4.5:( 60 , 0 , 41 , 1635),
          5:( 65 , 0 , 45 , 1655)
           }
        self.Vcmd = 1.5   #打死速度



        carPose = self.odom.pose.pose
        carVel  = self.odom.twist.twist

        self.cmd_vel.linear.x = 1500
        self.cmd_vel.linear.z = 1  
        self.cmd_vel.angular.z = 90 


        #距离控制速度
        if len(self.scan_date.ranges):
            min=self.isForward_hasObstacle(0,self.scan_date.ranges) 
            if self.eta_pre>-30 and self.eta_pre<30:
                if   min>4.5:
                    self.speed_v = 5
                elif min >4:
                    self.speed_v = 4.5
                elif min>3.5:
                    self.speed_v = 4
                elif min>3:
                    self.speed_v = 3.5
                elif min>2.5:
                    self.speed_v = 3
                elif min>2:
                    self.speed_v = 2.5
                elif min>1.5:
                    self.speed_v = 2
                else:
                    self.speed_v = 1.5
            else:
                self.speed_v = 1.5
        else:
            self.speed_v = 1.5

        if self.speed_v>self.speed_max:
            self.speed_v = self.speed_max

        # if self.amcl_pose.pose.pose.position.x>19:

        if self.mux_msg.a >4.5:
            v = 5
        elif self.mux_msg.a>4:
            v = 4.5
        elif self.mux_msg.a>3.5:
            v = 4
        elif self.mux_msg.a>3:
            v = 3.5
        elif self.mux_msg.a>2.5:
            v = 3
        elif self.mux_msg.a>2:
            v = 2.5
        elif self.mux_msg.a>1.5:
            v = 2
        else:
            v = 1.5
        

        self.speed_kp = self.move_speed_pid[self.speed_v][0] ; self.speed_ki = self.move_speed_pid[self.speed_v][1] ; self.speed_kd = self.move_speed_pid[self.speed_v][2]
        pwm = self.move_speed_pid[self.speed_v][3]


        self.Lfw = self.move_steer_pid[v][3] 
        self.steer_kp = self.move_steer_pid[v][0] ; self.steer_ki = self.move_steer_pid[v][1] ; self.steer_kd = self.move_steer_pid[v][2] 

           
        if self.goal_received:
            eta = self.getEta(carPose)  
            self.eta_pre = eta*180/math.pi
            

            if self.foundForwardPt:
                self.steering_angle = -self.getSteeringAngle(eta) 
                
                if self.eta_pre>30:
                    self.cmd_vel.angular.z = 180
                elif self.eta_pre<-30:
                    self.cmd_vel.angular.z = 0
                else:
                    self.steer_error = self.eta_pre
                    self.steer_p = self.steer_error
                    self.steer_d = self.steer_error - self.steer_pre_error
                    pid_steer = self.steer_kp*self.steer_p + self.steer_kd*self.steer_d
                    print pid_steer,self.steer_kp,self.steer_kd,self.eta_pre

                    z = int(self.base_angle + self.steering_angle*self.steer_kp + pid_steer)
                    if z<0:
                        z=0
                    elif z>180:
                        z=180
                    self.cmd_vel.angular.z = z

                if not self.goal_reached:
                    
                    if self.eta_pre>30 or self.eta_pre<-30:
                        self.cmd_vel.linear.x = int(self.move_speed_pid[1.5][3])
                    else:
                        self.speed_error = self.speed_v - self.mux_msg.a
                        self.speed_p = self.speed_error
                        self.speed_d = self.speed_error - self.speed_pre_error
                        self.speed_pre_error = self.speed_error

                        pid_speed = self.speed_kp*self.speed_p + self.speed_kd*self.speed_d 
                        self.cmd_vel.linear.x = pwm + pid_speed
                    
                    self.cmd_vel.linear.z = 0
                    #防止pid突然启动车
                    if self.mux_msg.a <0.6:
                        pwm_lanuch = int(self.move_speed_pid[1.5][3])
                        self.cmd_vel.linear.x = int(pwm_lanuch)
        self.cmd_pub.publish(self.cmd_vel)

    #初始化visualization_msgs::Marker
    def init_marker(self):
        # self.points.header.frame_id = 'odom'
        # self.points.type = Marker.POINTS
        # self.points.action = Marker.ADD

        # #点标记  分别使用x和y比例作为宽度/高度
        # self.points.scale.x = 1 ; self.points.scale.y = 1 ;  self.points.scale.z = .0
        # #设置点为绿色
        # self.points.color.a = 1.0 ; self.points.color.r = .0 ; self.points.color.g = 1.0 ;self.points.color.b = .0
        # self.points.pose.orientation.x = .0 ; self.points.pose.orientation.y = .0 ; self.points.pose.orientation.z = .0 ; self.points.pose.orientation.w = .0
        # self.points.pose.position.x = .0 ; self.points.pose.position.y = .0 ; self.points.pose.position.z = .0 



        self.points.header.frame_id = self.line_strip.header.frame_id = self.goal_circle.header.frame_id = 'odom'
        self.points.ns = self.line_strip.ns = self.goal_circle.ns = 'Markers'
        self.points.action =self.line_strip.action =self.goal_circle.action = Marker.ADD
        self.points.pose.orientation.w = self.line_strip.pose.orientation.w = self.goal_circle.pose.orientation.w = 1.0
        self.points.id=0
        self.line_strip.id=1
        self.goal_circle.id=2

        self.points.type = Marker.POINTS;  self.line_strip.type = Marker.LINE_STRIP; self.goal_circle.type = Marker.CYLINDER

        #点标记  分别使用x和y比例作为宽度/高度
        self.points.scale.x = 0.1
        self.points.scale.y = 0.1
        #设置点为绿色
        self.points.color.g = 1.0
        self.points.color.a = 1.0

        #线条形 标记仅使用比例的X分量作为线条宽度
        self.line_strip.scale.x = 0.1
        #线为蓝色
        self.line_strip.color.b = 1.0
        self.line_strip.color.a = 1.0

        self.goal_circle.scale.x = self.goalRadius
        self.goal_circle.scale.y = self.goalRadius
        self.goal_circle.scale.z = 0.1
        #圆为黄色
        self.goal_circle.color.r = 1.0
        self.goal_circle.color.g = 1.0
        self.goal_circle.color.b = .0
        self.goal_circle.color.a = .5






    #如果该描述的x>0，意味该点在前进方向上。
    def isForwardWayPt(self,waypt,carPose):
        car2wayPt_x = waypt.x - carPose.position.x
        car2wayPt_y = waypt.y - carPose.position.y
        car_theta = self.getYawFromPose(carPose)

        car_car2wayPt_x =  math.cos(car_theta)*car2wayPt_x + math.sin(car_theta)*car2wayPt_y
        car_car2wayPt_y = -math.cos(car_theta)*car2wayPt_x + math.cos(car_theta)*car2wayPt_y

        if car_car2wayPt_x>0:
            return True
        else:
            return False

    #当该点在前进方向上时，要判断两点距离是否大于Lfw，如果大于Lfw，该点为前瞻点。
    def isWayPtAwayFromLfwDist(self,waypt,carPose_pos):
        dx = waypt.x - carPose_pos.x
        dy = waypt.y - carPose_pos.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < self.Lfw:
            return False
        else:
            return True

    #由汽车位姿得到当前偏航角  return yam
    def getYawFromPose(self,carPose):
        (r,p,y) = tf.transformations.euler_from_quaternion([carPose.orientation.x , carPose.orientation.y , carPose.orientation.z ,carPose.orientation.w])
        return y

    #return eta
    def getEta(self,carPose):
        odom_car2WayPtVec = self.get_odom_car2WayPtVec(carPose)
        eta = math.atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x)
        return eta

    #return dist2goal
    def getCar2GoalDist(self):
        carPose_pos = self.odom.pose.pose.position
        car2goal_x = self.odom_goal_pos.x - carPose_pos.x
        car2goal_y = self.odom_goal_pos.y - carPose_pos.y

        dist2goal = math.sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y)
        return dist2goal

    #return L1 ###########
    def getL1Distance(self,Vcmd):
        L1 = .0
        if Vcmd < 1.34 :
            L1 = 3/3.0
        elif  Vcmd > 1.34 and Vcmd < 5.36:
            L1 = Vcmd*2.24 / 3.0
        else:
            L1 = 12/3.0  
        return L1

    #return steeringAnge###############
    def getSteeringAngle(self,eta):
        steeringAnge=-math.atan2(self.L*math.sin(eta) , (self.Lfw/2+self.lfw*math.cos(eta)))*(180.0/math.pi)
        return steeringAnge


    #return odom_car2WayPtVec  返回汽车和线点之间连线的向量
    def get_odom_car2WayPtVec(self,carPose):
        carPose_pos = carPose.position
        carPose_yaw = self.getYawFromPose(carPose)
        odom_car2WayPtVec  = Point()
        forwardPt = Point()
        self.foundForwardPt = False

        if not self.goal_reached:
            for each_poseStamped in self.map_path.poses:
                map_path_pose = each_poseStamped  #取出路径下的第一个点然后break
                try:  
                    odom_path_wayPt = map_path_pose.pose.position
                    _isForwardWayPt = self.isForwardWayPt(odom_path_wayPt ,carPose)

                    if _isForwardWayPt:
                        _isWayPtAwayFromLfwDist = self.isWayPtAwayFromLfwDist(odom_path_wayPt , carPose_pos)
                        if _isWayPtAwayFromLfwDist:
                            forwardPt = odom_path_wayPt
                            self.foundForwardPt = True
                            break
                except Exception,e:
                    print e
                    print 'def get_odom_car2WayPtVec(self,carPose):  Exception!!!'
        elif self.goal_reached:
            forwardPt = self.odom_goal_pos
            self.foundForwardPt =False

        #清除rviz中的点和线
        self.points.points=[]
        self.line_strip.points=[]

        if self.foundForwardPt and (not self.goal_reached):
            self.points.points.append(carPose_pos)
            self.points.points.append(forwardPt)
            self.line_strip.points.append(carPose_pos)
            self.line_strip.points.append(forwardPt)

        self.marker_pub.publish(self.points)
        self.marker_pub.publish(self.line_strip)

        odom_car2WayPtVec.x =  math.cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.sin(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        odom_car2WayPtVec.y = -math.sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + math.cos(carPose_yaw)*(forwardPt.y - carPose_pos.y)
        return odom_car2WayPtVec



    def isForward_hasObstacle(self,th,laser):         # th 为int型
        th = int(th)
        pra = 5                 #检测角度周围范围 (th-pra,th+pra)
        distance_min = 1      #检测最小距离,单位 m
        stop_min=0.5           #刹车最小距离

        data=0                  #返回值

        th_max=th+pra+1
        th_min=th-pra
        if th_max<=0:                                       #角度范围全部为负
            Min= min(laser[360+th_min:360+th_max])
        elif th_min>=0:                                     #角度范围全部为正
            Min = max(laser[th_min:th_max])
        else:                                               #角度范围正负均有
            Min = min(min(laser[:th_max]),min(laser[360+th_min:360]))
        # print(Min)                                          #Min为th角度范围内距离最小值
        return Min



    def isForward_hasObstacle1(self,th,laser,distance_min=1):         # th 为int型
        th = int(th)
        pra = 5                 #检测角度周围范围 (th-pra,th+pra)
        distance_min = distance_min      #障碍物最小距离,单位 m
        th_max=th+pra+1
        th_min=th-pra
        if th_max<=0:                                       #角度范围全部为负
            Min= min(laser[360+th_min:360+th_max])
        elif th_min>=0:                                     #角度范围全部为正
            Min = min(laser[th_min:th_max])
        else:                                               #角度范围正负均有
            Min = min(min(laser[:th_max]),min(laser[360+th_min:360]))
        # print(Min)                                          #Min为th角度范围内距离最小值
        # 判断周围是否有障碍物，不需要注释即可
        left_min = min(laser[:30])
        right_min = min(laser[330:360])
        if left_min < distance_min or right_min < distance_min:
            if left_min < right_min:
                turn_tha_add=10-10*pow(left_min,2)
            else:
                turn_tha_add=10*pow(right_min,2)-10
        else:
            turn_tha_add=0
        return Min,turn_tha_add

if __name__=="__main__":
    rospy.init_node("Pursuit")
    pursuit=Pursuit()
    rospy.spin()