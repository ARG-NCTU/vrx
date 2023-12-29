#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool
import tf.transformations as tf_trans
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from pyexpat import model
from sensor_msgs.msg import Joy
from obstacle_detector.msg import Obstacles
import math 

class RealtoSimObstacle:
    def __init__(self):
        # self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        # self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.pub_state_to_mapgrid = rospy.Publisher("/reset_map", Bool, queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        self.timer_reset_model_state = rospy.Timer(rospy.Duration(0.02), self.timer_cb_reset)
        self.pub_obstacle_to_vr = rospy.Publisher("/obstacle_from_real",PoseStamped, queue_size=1)
        self.sub_obstacle = rospy.Subscriber('/raw_obstacles', Obstacles, self.obstacle_cb, queue_size=1)
        
        self.flag = False
        self.obstacle_name = 'real_obstacle_'
        self.max_num = 0
        self.existing_obstacle = []
        self.init_obstacle()
        self.joy = None
    
    def joy_callback(self, joy):
        if self.joy == None:
            self.joy = joy
            return
        joy_trigger = joy.buttons[4] and not self.joy.buttons[4]
        print('joy_trigger:', joy_trigger)
        
        if joy_trigger:
            self.flag = not self.flag 
            if self.flag == True: # reset done
                print('pub true')
                self.pub_state_to_mapgrid.publish(True)
            else:
                pass
            
        self.joy = joy
            
    def obstacle_cb(self, msg):
        self.obstacle_msg = msg

    def init_obstacle(self):
        for i in range (16):
            obstacle_pose = PoseStamped()
            model_name = self.obstacle_name + str(i)
            obstacle_pose.pose.position.x = 258
            obstacle_pose.pose.position.y = i + 2
            obstacle_pose.pose.position.z = -50 
            obstacle_pose.pose.orientation.x = 0
            obstacle_pose.pose.orientation.y = 0
            obstacle_pose.pose.orientation.z = 0
            obstacle_pose.pose.orientation.w = 1
            self.set_model(model_name = model_name, pose = obstacle_pose)
        
        # print(obstacle_pose)
        # clear existing obstacle
        self.max_num = 0
        self.existing_obstacle = []
        
     
    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose     
        self.pub_set_model_state.publish(model_state)
        if pose.pose.position.z == -50:
            return
        else:
            self.pub_obstacle_to_vr.publish(pose)

    def rm_original_obstacles(self, point):
        x = point[0]
        y = point[1]
        # elif self.ws == 2:
        top_boundary = 70
        bottom_boundary = 50
        left_boundary = 68
        right_boundary = -68
              

        #filter out original obstacles
        # print('x:', x, 'y:', y)
        if  bottom_boundary <= x <= top_boundary and \
            right_boundary <= y <= left_boundary :
            print('original obstacle')
            return False    
        
        else:
            return True


    def check_obstacle(self, new_model):
        threshold = 5
        try:
            if not self.existing_obstacle or len(self.existing_obstacle[0]) != len(self.existing_obstacle[1]):
                print('no existing obstacle')
                self.existing_obstacle = new_model
                return 
            else:
                for j in range(len(new_model[0])):
                    obstacle_updated = False
                    for i in range(len(self.existing_obstacle[0])):
                        dis = self.dist([self.existing_obstacle[0][i], self.existing_obstacle[1][i]], [new_model[0][j], new_model[1][j]])
                        if dis < threshold:  # update obstacle
                            # assume the obstacle got at the first time is correct 
                            ## obstacle pose updated
                            # self.existing_obstacle[0][i] = new_model[0][j]
                            # self.existing_obstacle[1][i] = new_model[1][j]
                            
                            obstacle_updated = True
                            break  # Stop searching once an obstacle is updated
                    if not obstacle_updated:
                        # Add new obstacle
                        self.existing_obstacle[0].append(new_model[0][j])  # x
                        self.existing_obstacle[1].append(new_model[1][j])  # y
                    
        except ValueError:
            pass
            
    def dist(self, p1, p2):
        dis =  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dis   

    def generate_points(self, x, y, order):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.5
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        model_name = self.obstacle_name + str(order)
        
        self.set_model(model_name = model_name, pose = pose)
        
    def timer_cb_reset(self, event):
        if self.flag == False:
            self.init_obstacle()   
            print('reset map')
            self.existing_obstacle = []
            return
        else:
            pass
        
    def timer_callback(self, event):            
        if self.flag == True:
            new_model = [[], []]
            for circle in self.obstacle_msg.circles:
                x = circle.center.x
                y= circle.center.y

                #remove original obstacles if detected
                if self.rm_original_obstacles([x,y]):
                    # model_name = self.obstacle_name + str(cnt)
                    new_model[0].append(x)
                    new_model[1].append(y)
                else: 
                    pass
  
            # update obstacle pose and add new obstacle
            print(new_model)
            self.check_obstacle(new_model) 
            print('existing obstacle:', len(self.existing_obstacle[0]))
            
            for i in range (len(self.existing_obstacle[0])):
                # print('i:', i)
                self.generate_points(self.existing_obstacle[0][i], self.existing_obstacle[1][i], i)
                self.max_num = len(self.existing_obstacle[0])
            # print('existing_obstacle:', self.existing_obstacle)
        else:
            pass

if __name__ == "__main__":
    rospy.init_node("real_to_sim_obstacle")
    real_to_sim_transform = RealtoSimObstacle()
    rospy.spin()