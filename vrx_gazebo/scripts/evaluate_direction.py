#! /usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest
import csv
import math 
import time
import os

class eval_direction(object):
    def __init__(self):
        self.fram_name = rospy.get_param("~frame_name", "wamv/base_link")
        self.paraent_name = rospy.get_param("~parent_name", "map")
        
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)
        self.sub_wamv = rospy.Subscriber("/wamv/truth_map_posestamped", PoseStamped, self.cb_wamv, queue_size=1)
        self.goal_dis = rospy.get_param("~goal_dis", 1)
        self.mission = rospy.get_param("~mission", "x")
        self.pub_cmd = rospy.Publisher("/wamv/cmd_vel", Twist, queue_size=1)
        self.model = GetModelStateRequest()
        self.auto = 0

        self.wamv_x, self.wamv_y, self.wamv_z = 0, 0, 0
        self.wamv_qx, self.wamv_qy, self.wamv_qz, self.wamv_qw = 0, 0, 0, 0
        self.init_wamv_qx, self.init_wamv_qy, self.init_wamv_qz, self.init_wamv_qw = 0, 0, 0, 0

        self.set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.objstate = SetModelStateRequest()
        self.stop_record = 0
        
        self.stop = Twist()
        self.stop.angular.x =0  
        self.stop.angular.y =0
        self.stop.angular.z =0
        self.stop.linear.x =0
        self.stop.linear.y =0
        self.stop.linear.z =0
        
        if self.mission == 'y':
            self.goal = [0, 0]
        elif self.mission == 'x':
            self.goal = [50, 50]
        else:
            pass 
        self.pub_cmd.publish(self.stop)
        self.time = None
        self.start_time = None
        self.inferece_time = 0
        # save param
        self.save_param_file = '/home/argrobotx/robotx-2022/catkin_ws/src/vrx/vrx_gazebo/scripts/fake_obstacle/csv/vrx/mission_' + self.mission + '_10.csv'
        self.creat_csv()
        self.set_wamv_pose()
        self.dis = 0
        
    def cb_joy(self, msg):
        start_btn = 7  
        end_btn = 6     
        self.stop_record = msg.buttons[5] 
        if (msg.buttons[start_btn] == 1 ) and (not self.auto):
            self.set_wamv_pose()
            rospy.loginfo('start mission')
            self.auto = 1
            self.start_time = time.time()
            
        if(msg.buttons[end_btn] == 1 ) and (self.auto == 1):
            self.pub_cmd.publish(self.stop)
            self.auto = 0
            rospy.loginfo('stop mission')
            
    def cb_wamv(self, msg):
        self.wamv_x = msg.pose.position.x
        self.wamv_y = msg.pose.position.y
        self.wamv_z = msg.pose.position.z
        self.wamv_qx = msg.pose.orientation.x
        self.wamv_qy = msg.pose.orientation.y
        self.wamv_qz = msg.pose.orientation.z
        self.wamv_qw = msg.pose.orientation.w
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    def creat_csv(self):
        os.makedirs(os.path.dirname(self.save_param_file), exist_ok=True)
        rospy.loginfo('creat csv file, path: %s', self.save_param_file)
        with open(self.save_param_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['X', 'Y', 'Z', 'orentation','dis_to_goal', 't'])
  
    def save_csv(self):
        with open(self.save_param_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.wamv_x, self.wamv_y, self.wamv_z,[self.wamv_qx, self.wamv_qy, self.wamv_qz, self.wamv_qw] ,self.dis, self.time])


    def set_wamv_pose(self):
        self.objstate.model_state.model_name = 'wamv'
        self.objstate.model_state.pose.position.x = 0
        self.objstate.model_state.pose.position.y = 50
        self.objstate.model_state.pose.position.z = -0.090229
        
        self.objstate.model_state.pose.orientation.x = 0
        self.objstate.model_state.pose.orientation.y = 0
        self.objstate.model_state.pose.orientation.z = 0
        self.objstate.model_state.pose.orientation.w = 1
        
        if self.mission == 'x':
            self.objstate.model_state.pose.orientation.w = 1
        elif self.mission == 'y':
            self.objstate.model_state.pose.orientation.z = -math.sin(math.radians(90/2))
            self.objstate.model_state.pose.orientation.w = math.cos(math.radians(90/2))
            
        self.objstate.model_state.twist.linear.x = 0
        self.objstate.model_state.twist.linear.y = 0
        self.objstate.model_state.twist.linear.z = 0
        self.objstate.model_state.twist.angular.x = 0
        self.objstate.model_state.twist.angular.y = 0
        self.objstate.model_state.twist.angular.z = 0
        self.set_model(self.objstate)
        rospy.loginfo('set wamv pose as mission %s', self.mission)
        
    def inference(self):   
        
        if self.goal is None:
            rospy.loginfo("No goal")
            return
        if self.auto == 0:
            return
        self.dis = math.sqrt((self.goal[0]-self.wamv_x)**2+(self.goal[1]-self.wamv_y)**2)

        # if self.dis < self.goal_dis:
        #     rospy.loginfo("goal reached")
        #     self.stop_record = 1
        #     self.goal = None
        #     cmd = Twist()
        #     cmd.linear.x = 0
        #     cmd.angular.z = 0
        #     self.pub_cmd.publish(cmd)
        #     return
       
        if self.mission == 'y'and self.wamv_y < 0 :
            rospy.loginfo("goal reached")
            self.stop_record = 1
            self.goal = None
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            self.pub_cmd.publish(cmd)
            return
        elif self.mission == 'x' and self.wamv_x > 50:
            rospy.loginfo("goal reached")
            self.stop_record = 1
            self.goal = None
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = 0
            self.pub_cmd.publish(cmd)
            return
        else:
            pass
            
        cmd = Twist()
        cmd.linear.x = 0.45
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0 #-0.45
        self.pub_cmd.publish(cmd)
        
        self.inferece_time = time.time() - self.start_time
        print("inferece time: ", self.inferece_time)
        rospy.loginfo ("dis: %f", self.dis)
        self.save_csv()

    def run(self): 
        while not rospy.is_shutdown():
            self.inference()
            rospy.sleep(0.1) 
            if (self.stop_record == 1):
                rospy.loginfo('stop record')
                rospy.loginfo('save file in %s', self.save_param_file)
                break  # Exit the loop


if __name__ == "__main__":
    rospy.init_node("eval_direction")
    eval_direction = eval_direction()
    try:
        eval_direction.run()
    except rospy.ROSInterruptException:
        pass 
