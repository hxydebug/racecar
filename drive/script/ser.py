#!/usr/bin/env python
#coding=utf-8

import serial
import serial.tools.list_ports
import struct
class Serial:
    def __init__(self,port,bps=38400,timex=0.1):#port端口号，bps波特率默认38400,超时时间timex默认0.1s
        try:
            self.ser=serial.Serial(port, bps, timeout=timex)
            self.odom_left=0
            self.odom_right=0
            self.turn_tha = 0
        except:
            print("can't open port:"+port)
    # def send_pwm(self,speed,angle):                             #speed和angle都是int型
    #     speed_L=speed%256
    #     speed_H=speed//256
    #     angle_L=angle%256
    #     angle_H=angle//256
    #     data=[0x00,0xAA,0xDC,0x05,0xDC,angle_H,0xC2,0x55]
    #     # data=[0x00,0xAA,speed_L,speed_H,angle_L,angle_H,0xC2,0x55]
    #     self.ser.write(data)
    #     print data
    #     # print(self.ser.read(8).hex())
    def send_pwm(self,speed,angle):                             #speed和angle都是int型
        if speed>2500:
            speed=2500
        elif speed<500:
            speed=500

        if angle>2500:
            angle = 2500
        elif angle<500:
            angle = 500

        speed_L=speed%256
        speed_H=speed//256
        angle_L=angle%256
        angle_H=angle//256
        data=[0x00,speed_L,speed_H,angle_L,angle_H,0x55]
        self.ser.write(data)
        # print(self.ser.read(8).hex())

    # def get_odom(self):                                         #得到编码器的值，self.odom_left self.odom_right,错误返回False
    #     self.ser.write('e'.encode('gbk'))
    #     odom=self.ser.read(6)
    #     if len(odom)==6:
    #     # try:
    #         if ord(odom[2])//128:
    #             self.odom_left = -(256-ord(odom[0]) + (255-ord(odom[1])) * 256 + (255-ord(odom[2])) * 65536)
    #         else:
    #             self.odom_left=ord(odom[0])+ord(odom[1])*256+ord(odom[2])*65536
    #         # print(ord(odom[5]))
    #         if ord(odom[5])//128:
    #             self.odom_right = -(256-ord(odom[3]) + (255-ord(odom[4])) * 256 + (255-ord(odom[5])) * 65536)
    #         else:
    #             self.odom_right=ord(odom[3])+ord(odom[4])*256+ord(odom[5])*65536
    #         # self.turn_tha=5.0*ord(odom[6])/255

    #         return True
    #     else:
    #         return False
    #     # except Exception,e:
    #     #     print e
    #         #     return False



    def get_odom(self):                                         #得到编码器的值，self.odom_left self.odom_right,错误返回False
        self.ser.write('e'.encode('gbk'))
        odom=[]
        dat='0'
        i=0
        
        try:
            # mes=self.ser.read(7)
            
            while (ord(dat)!=170 and i<8):
                mes=self.ser.read()
                
                dat=mes[0]
                odom.append(dat)
                # print(ord(odom[6]))
                i+=1
                
                    
            if len(odom)==2 or len(odom)==5:
                dat='0'
                while ord(dat)!=170 and i<8:
                    mes=self.ser.read()

                    dat=mes[0]
                    i+=1
                    odom.append(dat)
                if len(odom)==2 or len(odom)==5:
                    dat='0'
                    while ord(dat)!=170 and i<8:
                        mes=self.ser.read()
                        dat=mes[0]
                        odom.append(dat)
                        i+=1
       
            if ord(odom[6])==170:
                if ord(odom[2])//128:
                    self.odom_left = -(256-ord(odom[0]) + (255-ord(odom[1])) * 256 + (255-ord(odom[2])) * 65536)
                else:
                    self.odom_left=ord(odom[0])+ord(odom[1])*256+ord(odom[2])*65536
                # print(ord(odom[5]))
                if ord(odom[5])//128:
                    self.odom_right = -(256-ord(odom[3]) + (255-ord(odom[4])) * 256 + (255-ord(odom[5])) * 65536)
                else:
                    self.odom_right=ord(odom[3])+ord(odom[4])*256+ord(odom[5])*65536
                # print self.odom_left,self.odom_right

                return True
            
            else:
                return False
        except:
            self.ser.flushInput()
            print "得不到编码器"
            return False

    def clean_odom(self):                                      #清空编码器的值，成功返回True错误返回False
        self.ser.write('r'.encode('gbk'))
        try:
            if self.ser.read()[0] =='T':
                return True
            else:
                return False
        except:
            self.ser.flushInput()
            print "清空编码器出错"
 
    def search_com(self):                                       #查找端口，返回列表self.port_list
        self.port_list = list(serial.tools.list_ports.comports())
        if(len(self.port_list)==0):
            return False
        else:
            return self.port_list
    def get_trun(self):
        self.ser.write('s'.encode('gbk'))
        dat='0'
        data=[]
        try:
            while(ord(dat)!=240):
                mes=self.ser.read()
                dat=mes[0]
                data.append(mes[0])
            if len(data)==2:
                self.turn_tha=5.0*ord(data[0])/255
                return True
            else:
                return False
        except:
            self.ser.flushInput()
            return False
