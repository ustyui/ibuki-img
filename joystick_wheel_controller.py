#!/usr/bin/env python
# -*- coding: utf-8 -*-
device_name = 'wheel'
dir_front = 5000

"ibuki utils"
import ibuki

"ros modules"
import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Joy

"UDP modules"
import socket
import math

max_steer, joy_forward = 6500,6000
min_steer, joy_back = 4500,4000

joint_default = ibuki.seperate_command_string(ibuki.default(device_name))
"INITIALIZATION:default must be defined!"
class Joystick(object):
    def __init__(self):
        self.joint_now = joint_default
        self.maxspeed = 2650
        self.key = None
        self.string = ''
        self.jointnow1, self.jointnow2, self.jointnow3 = 0,0,0   
        self.loop_rate = rospy.Rate(20)
        #Publisher
        self.pub = rospy.Publisher('wheel_info', String, queue_size = 10)
        self.pubcmd = rospy.Publisher('wheel_cmdmonitor', String, queue_size =25)
        #Subscriber
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.Subscriber('wheel_info',String, self.seperate_and_pose)
        #rospy.Subscriber('blink', String, self.keys_blink)              
                
    def callback(self, joy):
        if(joy.buttons[4] == 1):
            self.maxspeed = self.maxspeed - 50
            print("SPEEEEED DOWN")
        if(joy.buttons[5] == 1):
            self.maxspeed = self.maxspeed + 50
            print("SPEEEEED UP")
        self.direction = joy.axes[0]
        self.speed = joy.axes[1]
        if(self.speed>0):
            self.jointnow1 = 1000 + self.speed*self.maxspeed
        if(self.speed<0):
            self.jointnow1 = -1000 + self.speed*self.maxspeed
        self.jointnow2 = 5000+ 1000*self.direction
        self.jointnow3 = joy_forward
        "jointnow3"
        if self.jointnow1 < 0:
            self.jointnow1 = -self.jointnow1
            self.jointnow3 = joy_back
        elif self.speed == 0.0:
            self.jointnow3 = 5000
        #print(self.jointnow1, self.jointnow2,self.jointnow3)
        finalmessage = self.merge_them()
        print(finalmessage)
        wheelsock.sendto(finalmessage, (ibuki.ip(device_name), ibuki.port(device_name)))
        
    
    def start(self):
        rospy.loginfo("In attesa")
        
        while not rospy.is_shutdown():
            #message = ibuki.merge_them(self.joint_now)
            #print "hello"
            rtclient.sendto('hello', (ibuki.ip(device_name),ibuki.port('wheel2')))
            data_client, addr_rt = rtclient.recvfrom(1024)
            #print data_client
            self.pub.publish(data_client)
            self.pubcmd.publish(self.merge_them())
            #self.pub.publish(message)
            self.loop_rate.sleep()
            
    def seperate_and_pose(self, msg):
        _string = msg.data
        _list_angle = [0,0,0]
        if _string.startswith('a'):
            _list_angle = _string.split('a')
        _list_angle[1] = int(_list_angle[1])
        _list_angle[2] = int(_list_angle[2])
        
        #return _list_angle 
        angle_now = ([_list_angle[1], _list_angle[2]])
        #print(angle_now)
#        if (angle_now[0] < 80) or (angle_now[0] >350):
#            self.just_change_linear(6000, 0, device_name, wheelsock)
#        elif (angle_now[0] < 170):
#            self.just_change_linear(5000, 0, device_name, wheelsock)
#        elif (angle_now[0] <260):
#            self.just_change_linear(6000, 0, device_name, wheelsock)
#        elif (angle_now[0] <350):
#            self.just_change_linear(5000, 0, device_name, wheelsock)
        "command string of linear actuator ,e.g. '05xx0'"
        this_angle = int(5000 + 1000*math.sin(2*angle_now[0]*2*math.pi/360))
        self.just_change_linear(this_angle, 0, device_name, wheelsock)
        
        print angle_now
        return angle_now
    def just_change_linear(self, _nb, order, _device_name, _socket):
        self.joint_now[order] = _nb
        _message_send = self.merge_them()
        _socket.sendto(_message_send, (ibuki.ip(_device_name), ibuki.port(_device_name)))  
        
    def merge_them(self):
        _message = ''
        _joint_send=  []
        self.joint_now[2] = int(self.jointnow1)
        self.joint_now[1] = int(self.jointnow2)
        self.joint_now[3] = int(self.jointnow3)
        for them in range(len(self.joint_now)):
            _joint_send.append(str(self.joint_now[them]).zfill(5))
        _message = _message.join(_joint_send)
        _message.replace(" ","")
        print(_message)
        return _message
        
    def fake(self,msg):
        print('hello')
        
            
if __name__ == "__main__":
    "motor command socket"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    wheelsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
    #motorsock.bind((ip_master, port_drive_headl))
    
    "Send To Mbed"
    #motorsock.sendto(message_init, (ibuki.ip(device_name), ibuki.port(device_name)))
    rospy.init_node('joystick_'+device_name)
    
    joystickA = Joystick()
    joystickA.start()
    
