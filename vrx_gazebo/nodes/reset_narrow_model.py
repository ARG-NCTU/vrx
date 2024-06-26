#!/usr/bin/env python3
from re import X
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import  Joy
import struct
import roslib
import pickle
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from datetime import datetime
import subprocess
import yaml
from std_msgs.msg import Header,String
from nav_msgs.msg import Odometry
import random
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
import math
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState


class robot_status(object):
    def __init__(self):

        # with open('duckiepond-devices-machine.yaml', 'r') as f:
        #     data = yaml.load(f)

        # self.machine,self.compute_unit = self.who_am_I(data)
        self.machine = "wamv"
        self.boat = "wamv"
        self.boat1 = "wamv"
        self.r = 8
        self.flag = "finish"

        self.collision_states = False

        self.reset_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # self.sub_collision1 = rospy.Subscriber("/"+self.machine+"1/bumper_states", ContactsState, self.cb_collision1, queue_size=1)
        # self.sub_collision2 = rospy.Subscriber("/"+self.machine+"2/bumper_states", ContactsState, self.cb_collision2, queue_size=1)
        # self.sub_collision3 = rospy.Subscriber("/"+self.machine+"3/bumper_states", ContactsState, self.cb_collision3, queue_size=1)
        # self.sub_collision4 = rospy.Subscriber("/"+self.machine+"4/bumper_states", ContactsState, self.cb_collision4, queue_size=1)

        self.pub_status = rospy.Publisher( "/experinment_state", String, queue_size=1)
        self.pub_status_int = rospy.Publisher( "/experinment_state_int", Float32, queue_size=1)

        self.state = Float32()
        self.reach_goal = 0
        self.reset_start()
        self.path = '/home/argrobotx/robotx-2022/catkin_ws/src/vrx/vrx_gazebo/nodes/point.txt'
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)



        #print(self.robot_status.data)


    def cb_publish(self, event):
        print("reset")
        # self.pub_status_int.publish(self.state)
        # self.pub_status.publish(self.flag)
        # if self.flag == "collision":
        #     #f = open(self.path, 'a')
        #     #f.write(str(self.reach_goal))
        #     self.reset()
        #     self.state.data = 1
        #     self.pub_status_int.publish(self.state)
        #     self.state.data = 0
        #     self.pub_status_int.publish(self.state)
        #     self.flag = "start"
        # elif self.flag == "finish":
        #     self.reset()
        #     self.flag = "start"

    # def target(self,msg):
    #     count = 0
    #     for i in range (20):
    #         if msg.data_flag[i]== 2:
    #             count = count + 1

    #     print(count)
    #     print(self.reach_goal)
    #     if count > self.reach_goal:
    #         f = open(self.path, 'a')
    #         f.write(str(self.reach_goal))
    #     self.reach_goal = count
    #     if count == 20:
    #         self.flag = "finish"


    # def cb_collision1(self, msg):
    #     if self.collision_states == True:
    #         if msg.states == [] and self.count > 2000:
    #             self.collision_states = False
    #             # print(self.collision_states)
    #         else:
    #             self.count += 1
    #             # print(self.count)
    #     elif msg.states != [] and self.count == 0:
    #         self.collision_states = True
    #         self.flag = "collision"
    #         #print("collsion time: ", self.collision)
    #         #print(self.collision_states)
    #     else:
    #         self.collision_states = False
    #         self.count = 0

    # def cb_collision2(self, msg):
    #     if self.collision_states == True:
    #         if msg.states == [] and self.count > 2000:
    #             self.collision_states = False
    #             # print(self.collision_states)
    #         else:
    #             self.count += 1
    #             # print(self.count)
    #     elif msg.states != [] and self.count == 0:
    #         self.collision_states = True
    #         self.flag = "collision"
    #         #print("collsion time: ", self.collision)
    #         #print(self.collision_states)
    #     else:
    #         self.collision_states = False
    #         self.count = 0
    
    # def cb_collision3(self, msg):
    #     if self.collision_states == True:
    #         if msg.states == [] and self.count > 2000:
    #             self.collision_states = False
    #             # print(self.collision_states)
    #         else:
    #             self.count += 1
    #             # print(self.count)
    #     elif msg.states != [] and self.count == 0:
    #         self.collision_states = True
    #         self.flag = "collision"
    #         #print("collsion time: ", self.collision)
    #         #print(self.collision_states)
    #     else:
    #         self.collision_states = False
    #         self.count = 0

    # def cb_collision4(self, msg):
    #     if self.collision_states == True:
    #         if msg.states == [] and self.count > 2000:
    #             self.collision_states = False
    #             # print(self.collision_states)
    #         else:
    #             self.count += 1
    #             # print(self.count)
    #     elif msg.states != [] and self.count == 0:
    #         self.collision_states = True
    #         self.flag = "collision"
    #         #print("collsion time: ", self.collision)
    #         #print(self.collision_states)
    #     else:
    #         self.collision_states = False
    #         self.count = 0
             
    def reset(self): 
        state_msg = ModelState()
        state_msg.model_name = "wamv1"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -660
        state_msg.pose.position.y = -40
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)  

        state_msg = ModelState()
        state_msg.model_name = "wamv2"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -650
        state_msg.pose.position.y = -30
        state_msg.pose.position.z = -0
        self.reset_model(state_msg) 

        state_msg = ModelState()
        state_msg.model_name = "wamv3"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -650
        state_msg.pose.position.y = -50
        state_msg.pose.position.z = 0
        self.reset_model(state_msg) 

        state_msg = ModelState()
        state_msg.model_name = "wamv4"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -640
        state_msg.pose.position.y = -40
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)


        state_msg = ModelState()
        state_msg.model_name = "wamv5"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -650
        state_msg.pose.position.y = -40
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)
                     

        #print(self.robot_status.data[1])     
        # 
        # 
    def reset_start(self): 
        state_msg = ModelState()
        state_msg.model_name = "wamv1"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -570
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)  

        state_msg = ModelState()
        state_msg.model_name = "wamv2"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -560
        state_msg.pose.position.y = 10
        state_msg.pose.position.z = -0
        self.reset_model(state_msg) 

        state_msg = ModelState()
        state_msg.model_name = "wamv3"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -560
        state_msg.pose.position.y = -10
        state_msg.pose.position.z = 0
        self.reset_model(state_msg) 

        state_msg = ModelState()
        state_msg.model_name = "wamv4"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -550
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)

        state_msg = ModelState()
        state_msg.model_name = "wamv5"
        
        # quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        state_msg.pose.position.x = -560
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        self.reset_model(state_msg)
                     
  





    def who_am_I(self,data):
            ret_byte = subprocess.check_output(['ifconfig'])
            ret_str = ret_byte.decode('utf-8')
            # Cut string from 'equal symbol' to 'degree C symbol', then convert to float
            en = ret_str[ret_str.find('eno1:'): ret_str.find('192.168.1.255')]
            ip = en[en.find('inet')+5: en.find('netmask')-2]
            machine,device = self.get_key(data,ip)
            return machine,device

    def get_key(self,dict,value):
        for k, v in dict.items():
            for v,v1 in v.items():
                for v1,v2 in v1.items():
                    if v2 == value:
                        return k,v






if __name__ == "__main__":
	rospy.init_node("robot_status")
	robot_status = robot_status()
	rospy.spin()
