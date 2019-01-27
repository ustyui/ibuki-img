#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, math
import socket
import numpy as np

import rospy
from std_msgs.msg import String
#
# IMPORTANT NOTE: Don't add a coding line here! It's not necessary for
# site files
#
# IBUKI MODULE 2.0
#

#==============================================================================
# IBUKI'S CONTROLLER IP LIST
# 
# The definition of ibuki Raspis, mbeds ip
# There are three Raspis, ten mbeds in use in Ibuki testtype
# there is no need to change anything here if no additional device is added
#==============================================================================
ip_dict = {'master':"192.168.99.101",
           'ibuki2':"192.168.99.102",
           'ibuki3':"192.168.99.103",
           'handr':"192.168.99.2",
           'handl':"192.168.99.3",
           'armr':"192.168.99.12",
           'arml':"192.168.99.13",
           'neck':"192.168.99.22",
           'headl':"192.168.99.31",
           'headc':"192.168.99.32",
           'headr':"192.168.99.33",
           'hip':"192.168.99.42",
           'wheel':"192.168.99.52",
           'codex':"192.168.99.99",
           'test':"119.63.197.151"
}
#==============================================================================
# IBUKI'S CONTROLLER PORT LIST
# 
# The definition of ibuki Raspis, mbeds port
# wheel controller uses two ports
# there is no need to change anything here if no additional device is added
#==============================================================================
port_dict = {'handl':10006,
             'handr':10007,
             'tts':10008,
             'tts2':10009,
             'tts3':10010,
             'headl':10011,
             'headc':10012,
             'headr':10013,
             'arml':10014,
             'armr':10015,
             'neck':10016,
             'hip':10017,
             'wheel':10018,
             'wheel2':10019,
             'executor':10099,
             'test':56
}
#==============================================================================
# IBUKI'S CONTROLLER DEFALULT POSITION LIST
# 
# The definition of joint positions of every controller
# protocol : |0xxxx|0xxxx|0xxxx|0xxxx|0xxxx| xxxx is joint position*1000
# please be careful when you change the values
# TODO: change it to tuples or uneditable lists
#==============================================================================
default_dict = {'handl':'0437304104042240389604015',
                'handr':'0604505925059250574605537',
                'headl':'035000430042000410005900',
                'headc':'0340006300049000485004000',
                'headr':'0410005400045000600004790',
                'arml':'0730005000050000500005000',
                'armr':'0270004600045000630005300',
                'neck':'0800003000048300500005100',
                'hip':'0500004871052360500005000',
                'wheel':'0500005000010000500004000',
                'test':'0500005000050000500005000'
}
#==============================================================================
# IBUKI'S POSTURES LIST
# 
# ibuki's posture
# TODO: enable the function of perform mapping destination
#==============================================================================
pose_dict = {'rightarm':'0600005000050000500005000'}

"socket initialization"
motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def ip(__devicename):
    return ip_dict[__devicename]

def port(__portname):
    return port_dict[__portname]
    
def default(__devicename):
    return default_dict[__devicename]

def pose(__posename):
    return pose_dict[__posename]

#==============================================================================
# PATTERN GENERATOR
# 
# read .ysq file and return motion-time and timing-time (list)
#==============================================================================
def pattern_generator(_filename):
    "read .ysq file"
    motion_file = open('/root/ibuki_ws/src/ibuki_motion/src/symposium/'+_filename,'r')
    motion_list = []
    
    "get list of timing and motion [('timing','motion')]"
    for line in motion_file.readlines():
        line = line.strip()
        #print(line)
        motion_line = line.split('\t')
        motion_list.append(motion_line)
        #print(motion_line)
        
    #print(motion_list)
        
    "get time sequences"
    temp_timing = []
    inte_timing = []
    #same effect with lambda function
    for index in range(0,len(motion_list)):
        temp_timing.append(motion_list[index][0])
    
    for index in range(0,len(temp_timing)):
        temp_timing[index]=float(temp_timing[index])
        if (index == 0):
            inte_timing.append(temp_timing[index])
        else:
            inte_timing.append(temp_timing[index]-temp_timing[index-1])
    
    #print(inte_timing)
    
    "get motion sequences"
    motion_pattern = []
    #emotion_sign = []
    
    for index in range(0,len(motion_list)):
        motion_pattern.append(motion_list[index][1])
        
#    for index in range(0,len(motion_list)):
#        emotion_sign.append(motion_list[index][2])
    "what return is two lists: motion-time and timing-time."        
    return motion_pattern, inte_timing#, emotion_sign

#==============================================================================
# MOTION GENERATOR
# 
# read .ysq file, generate mapping motion to the correct controller
#
# mainly for stable demo, not recommened when you are doing research
#
# the weak point of .ysq file is, the input is completely map command,so
# if default value is changed, your .ysq file would be wholely shift from 
# the new default. 
# TODO: a new kind of file is needed, or firstly develop the function of 
# shifting .ysq file
#==============================================================================
def motion_generator(_filename,_devicename):
    
    "socket initialization"
    #motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message_init = default(_devicename)
    "Timer"
    start_clk = time.time() #actual time of running start
    "THRESHOLD: USUALLY WE DON'T NEED PRECISE SYNC BUT IF PLEASE KEEP EYE ON"
    inte_threshold = 0.005
    generated_motion_pattern = []
    interval_timing = []
    #empty = []
    "PATTERN GENERATOR"
    generated_motion_pattern, interval_timing = pattern_generator(_filename)
    #print(start_clk)
    for index in range(0,len(interval_timing)):
    
        #motorsock.sendto(message_init,(ip(_devicename), port(_devicename)))
            
        "Make correct timing for each motion, MODE:lay-back"
        if (index == 0):
            motorsock.sendto(message_init,(ip(_devicename), port(_devicename)))
            time.sleep(inte_threshold)
                
        time.sleep(interval_timing[index])  
            
        "Change to motor command encode"
        message_motor = generated_motion_pattern[index]
        print(message_motor)
        motorsock.sendto(message_motor,(ip(_devicename), port(_devicename)))
            
    end_clk = time.time()
    print('[RUNNING_TIME] ',end_clk-start_clk)
    
#==============================================================================
# EXECUTE MOTOR (ROUND-TRIP)
# 
# enable the a specific controller, and control the joints to a specific 
# position, and keep it for a time, and then return to the default position
#
# CAUTION: WE DON'T CONTROL SPEED IN THIS FUNCTION!!!
#
# usually used in fast motion(mouth, finger, blink)
# So the interval_time should not be set more than 1 sec
#==============================================================================
def exec_motor(message_motor, _devicename, interval_time = 0.2): 
    "Timer"
    start_clk = time.time() #actual time of running start
    "Threshold: Because there is a delay when Ibuki due playing .wav"
    #print(start_clk)
    message_init = default(_devicename)
    
    print(message_motor)
    
    motorsock.sendto(message_motor, (ip(_devicename), port(_devicename)))
    time.sleep(interval_time*0.5)
    motorsock.sendto(message_init,(ip(_devicename), port(_devicename)))
    time.sleep(interval_time*0.5)
    end_clk = time.time()
    print('runningtime:', end_clk-start_clk)

#==============================================================================
# RESET
# 
# reset to default position 
# nothing good to say
# TODO: build a slow reset system, mbed controller feedback, new ports needed
#==============================================================================
def reset(_devicename):
    message_init = default(_devicename)
    motorsock.sendto(message_init,(ip(_devicename), port(_devicename)))
    print('reset to ',message_init)
    #time.sleep(interval_time*0.5)

#==============================================================================
# MERGE FUNCTION OF IBUKI
# 
# merge the int list [x,xx,xxx,xxxx,xxxx] into a complete string
# return sendable mbed command string
# the INPUT must be in global use
#==============================================================================
def merge_them(_global_joint_now):
    _message = ''
    _joint_send = []
    for them in range(len(_global_joint_now)):
        _joint_send.append(str(_global_joint_now[them]).zfill(5))
    _message = _message.join(_joint_send)
    _message.replace(" ","")
    #print(_message)
    return _message

#==============================================================================
# SEPERATE COMMAND STRING
# 
# seperate the command string ["0xxxx0xxxx0xxxx0xxxx0xxxx"] to 
# an int list [xxxx,xxxx,xxxx,xxxx,xxxx]
#==============================================================================
def seperate_command_string(_command):
    list_command =[]
    for index in range(0,5):
        list_command.append(int(_command[index*5:index*5+5]))
    return list_command
    
#==============================================================================
# SEPERATE ANGLES
# 
# usually used in reading value of mbed sensors
# protocal: 'axxaxxxa' xx is sensor value (int)
# return list of two sensor data
# TODO: increase the usablility
#==============================================================================
def seperate_angles(_rosmessage):
    if type(_rosmessage.data) != int:
        
        _string = _rosmessage.data
        _list_angle = [0,0,0]
        if _string.startswith('a'):
            _list_angle = _string.split('a')
        #why I chose from 1? because 0 is an empty value
        _list_angle[1] = int(_list_angle[1])
        _list_angle[2] = int(_list_angle[2])
        
        print ([_list_angle[1], _list_angle[2]])
        return [_list_angle[1], _list_angle[2]]

#==============================================================================
# JOINT TO WHERE
# 
# THIS FUNCTION USE .YSQ FILE
# use joint_now (global) to move the joints to a specific position linearlly
# joint_now is also changed, although we don't get any feedback
# depends on the precision of PID controll of that controller
#==============================================================================
def joint_to_where(_joint_now,_filename,_devicename):
    "np the joint_now"
    _joint_now_np = np.array(_joint_now)
    "open the .ysq file"
    #old_timing = 0
    new_timing = 0
    target_motion_pattern = []
    time_needed = []
    #emotion = []
    start_clk = time.time() #actual time of running start
    target_motion_pattern, time_needed = pattern_generator(_filename)
    "change the motion command to the joint position list"
    for i in range(len(target_motion_pattern)):
        "init"
        mrg_to_target = _joint_now_np
        
        list_target_pos = seperate_command_string(target_motion_pattern[i])
        new_timing = time_needed[i]
        "where is joint now? compare it with the nst target position"
        mrg_to_target = np.array(list_target_pos) - mrg_to_target
        "how long is the time needed, generate the steps"
        inteval_jtw = 0.05
        time_inteval_jtw = new_timing# - old_timing
        lines_nb = int(time_inteval_jtw/inteval_jtw)
        #old_timing = new_timing
        step_to_target = mrg_to_target/lines_nb
        #step_to_targetint = list(np.array(step_to_target,dtype = 'int32'))
        "for line = 0 to line = maxline, add the step gradully"
        for lines in range(0, lines_nb):
            _joint_now_np = _joint_now_np + step_to_target
            _joint_now_int = list(np.array(_joint_now_np,dtype = 'int32'))
            _message_ = merge_them(_joint_now_int)
            "send the signal to the correct device"
            motorsock.sendto(_message_, (ip(_devicename),port(_devicename)))
            time.sleep(inteval_jtw)
            print(_message_)    
        "deal with the joint now lists"
        end_clk = time.time()
        print('[RUNNING_TIME] ',end_clk-start_clk)
    _joint_now = list(_joint_now_np)
    return _joint_now
  
#==============================================================================
# JOINT TO THERE
# 
# use joint_now (global) to move the joints to a specific position linearlly
# joint_now is also changed, although we don't get any feedback
# depends on the precision of PID controll of that controller
#==============================================================================   
def joint_to_there(_joint_now,_joint_there,_time_needed,_devicename):
    new_timing = 0
    start_clk = time.time() #actual time of running start
    "np the joint_now"
    _joint_now_np = np.array(_joint_now)
    target_motion_pattern, time_needed = _joint_there, _time_needed
    mrg_to_target = _joint_now_np
        
    list_target_pos = seperate_command_string(target_motion_pattern)
    new_timing = time_needed
    "where is joint now? compare it with the nst target position"
    mrg_to_target = np.array(list_target_pos) - mrg_to_target
    "how long is the time needed, generate the steps"
    inteval_jtw = 0.05
    time_inteval_jtw = new_timing# - old_timing
    lines_nb = int(time_inteval_jtw/inteval_jtw)
    #old_timing = new_timing
    step_to_target = mrg_to_target/lines_nb
    #step_to_targetint = list(np.array(step_to_target,dtype = 'int32'))
    "for line = 0 to line = maxline, add the step gradully"
    for lines in range(0, lines_nb):
        _joint_now_np = _joint_now_np + step_to_target
        _joint_now_int = list(np.array(_joint_now_np,dtype = 'int32'))
        _message_ = merge_them(_joint_now_int)
        "send the signal to the correct device"
        motorsock.sendto(_message_, (ip(_devicename),port(_devicename)))        
        time.sleep(inteval_jtw)
        print(_message_)    
    "deal with the joint now lists"
    end_clk = time.time()
    print('[RUNNING_TIME] ',end_clk-start_clk)
    _joint_now = list(_joint_now_np)
    
    return _joint_now
    
    
"positionname is a string"
def to_position(_pos_name, _dev_name):
    motorsock.sendto(_pos_name, (ip(_dev_name), port(_dev_name)))
    
    
#==============================================================================
# JOINT
# 
# class Joint 
# joint_now is also changed, although we don't get any feedback
# depends on the precision of PID controll of that controller
#==============================================================================   
class Joint(object):
    def __init__(self):
        self.name = 'test'
        self.spinrate = 20
        self.threshold = [0,0,0,0]
        self.fromwhich = 2
        
        self.joint_default = seperate_command_string(default(self.name))
        self.joint_now = seperate_command_string(default(self.name))
        self.key = None
        self.loop_rate = rospy.Rate(self.spinrate)

        #Publisher
        self.pub = rospy.Publisher('info_'+self.name, String, queue_size = 25)
        #Subscriber
        rospy.Subscriber('keys', String, self.callback)
        rospy.Subscriber('peer', String, self.move_limb) #peer from PC
        #rospy.Subscriber('peer',, self.move_arml)                
        
    def callback(self, msg):
        self.key = msg.data
        if(msg.data == 'j'):
            print("hello")
            msg.data == 'm'
        elif(msg.data == 'r'):
            self.joint_now = joint_to_where(self.joint_now,'motion_'+self.name+'a.ysq',self.name)
        elif(msg.data == 't'):
            self.joint_now = joint_to_where(self.joint_now,'motion_'+self.name+'b.ysq',self.name)
        elif(msg.data == 'g'):
            self.pub.publish('stop please..')
        elif(msg.data == 'b'):
            reset(self.name)
    
    def start(self):
        rospy.loginfo(self.name)
        
        while not rospy.is_shutdown():
            #message = ibuki.merge_them(self.joint_now)
            #self.pub.publish(message)
            self.loop_rate.sleep()
        
    "in neck, _which = 0 and 1"
    def change_joint_now(self, sensor_value, _which = 2):
        self.joint_now[_which] = int(self.joint_default[_which]+ self.threshold[0] +\
        self.threshold[1]*math.sin(sensor_value*2*math.pi/360))
        self.joint_now[_which+1] = int(self.joint_default[_which + 1] + self.threshold[2] +\
        self.threshold[3]*math.sin(sensor_value*2*math.pi/360))
        
    """UNIQUE FUNCITONS"""
    def move_limb(self, msg):
        
        list_sensor = seperate_angles(msg)
#        print(list_sensor)
        the_value_i_want = int(list_sensor[0]) + 90
        #print the_value_i_want
        self.change_joint_now(the_value_i_want,self.fromwhich)
        #print(self.joint_now)
        _message_wait = merge_them(self.joint_now)
        print _message_wait
        to_position(_message_wait,self.name)
        
    
#==============================================================================
# JOINT
# 
# class Joint 
# joint_now is also changed, although we don't get any feedback
# depends on the precision of PID controll of that controller
#==============================================================================   
class IdleJoint(object):
    def __init__(self):
        self.name = 'test'
        self.spinrate = 20
        self.whichkind = 0
        self.joint_default = seperate_command_string(default(self.name))
        self.joint_now = seperate_command_string(default(self.name))
        self.key = None
        self.loop_rate = rospy.Rate(self.spinrate)

        #Publisher
        self.pub = rospy.Publisher('info_'+self.name, String, queue_size = 25)
        #Subscriber
        rospy.Subscriber('keys', String, self.callback)
        if self.name == 'headl':
            rospy.Subscriber('blink', String, self.keys_blink)
        #rospy.Subscriber('peer',, self.move_arml)                
        
    def callback(self, msg):
        self.key = msg.data
        if(msg.data == 'j'):
            print("hello")
            msg.data == 'm'
        elif(msg.data == 'r'):
            self.joint_now = joint_to_where(self.joint_now,'motion_'+self.name+'a.ysq',self.name)
        elif(msg.data == 't'):
            self.joint_now = joint_to_where(self.joint_now,'motion_'+self.name+'b.ysq',self.name)
        elif(msg.data == 'g'):
            self.pub.publish('stop please..')
        elif(msg.data == 'b'):
            reset(self.name)
            
    def keys_blink(self, msg):
        print msg.data
        if(msg.data == 'blink'):
            exec_motor('0350005800052000410005900',self.name)
            msg.data == 'm'
    
    def start(self):
        rospy.loginfo(self.name)
        
        while not rospy.is_shutdown():
            #message = ibuki.merge_them(self.joint_now)
            #self.pub.publish(message)
            self.loop_rate.sleep()
        