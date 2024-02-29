#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool
import tf.transformations as tf_trans
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from pyexpat import model
from sensor_msgs.msg import Joy
from obstacle_detector.msg import Obstacles
import math 
from std_msgs.msg import Int8MultiArray


class RealtoSimObstacle:
    def __init__(self):
        
        # self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        # self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.sub_obstacle_map = rospy.Subscriber("/grid_map", Int8MultiArray, self.obstacle_map_cb)
        self.delete_model_service = '/gazebo/delete_model'
        rospy.wait_for_service(self.delete_model_service)
        self.delete_model_proxy = rospy.ServiceProxy(self.delete_model_service, DeleteModel)

        self.flag = False
        self.obstacle_name = 'real_obstacle_'
        self.pending_updates = [] 
        self.existing_obstacle = set() 
        self.init_obstacle()
        self.joy = None

        self.grid_width = 150
        self.grid_height = 150
        self.resolution = 1.0
        self.origin_x = 1400.0
        self.origin_y = -50.0
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=int)
        self.load_sdf()

    def load_sdf(self):
        sdf_file_path = "/home/argrobotx/robotx-2022/catkin_ws/src/vrx/vrx_gazebo/models/polyform_a7_greenglow/model.sdf"
        static_value = True
        try:
            with open(sdf_file_path, 'r') as file:
                sdf_content = file.read()

            if '<static>false</static>' in sdf_content and static_value:
                sdf_content = sdf_content.replace('<static>false</static>', '<static>true</static>')
            elif '<static>true</static>' in sdf_content and not static_value:
                sdf_content = sdf_content.replace('<static>true</static>', '<static>false</static>')
            elif '<model' in sdf_content and '<static>' not in sdf_content:
          
                sdf_content = sdf_content.replace('<model', '<model>\n  <static>{}</static>'.format('true' if static_value else 'false'), 1)
            self.obstacle_path = sdf_content
            
        except IOError as e:
            print("Error reading file: ", e)
            self.obstacle_path = None

    def joy_callback(self, joy):       
        if self.joy == None:
            self.joy = joy
            return
        # joy_trigger = joy.buttons[4] and not self.joy.buttons[4]
        joy_trigger = joy.buttons[2] and not self.joy.buttons[2]

        if joy_trigger:
            self.flag = True
            print(self.flag )

        elif joy.buttons[8]:
            self.flag = False
            print('Reset')

        self.joy = joy
            
    def obstacle_map_cb(self, msg):
        self.obstacle_msg = msg

    def map_grid_to_world(self):
        obstacle_poses_world = []
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.grid[y][x] == 1:
                    obstacle_x = self.origin_x + x * self.resolution + self.resolution / 2
                    obstacle_y = self.origin_y + y * self.resolution + self.resolution / 2
                    obstacle_poses_world.append((obstacle_x, obstacle_y))
        obs_len = len(obstacle_poses_world)
        return obstacle_poses_world

    def init_obstacle(self):
        for existing_obstacle in list(self.existing_obstacle):
            self.remove_obstacle(existing_obstacle)

    def remove_obstacle(self, model_name):
        self.delete_model(model_name)
        self.existing_obstacle.remove(model_name)

    def generate_points(self, x, y, order):
        pose = PoseStamped()
        pose.pose.position.x = x  
        pose.pose.position.y = y 
        pose.pose.position.z = -0.5
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        model_name = self.obstacle_name + str(x) + '_' + str(y)
        self.pending_updates.append((model_name, pose))
 

    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose.position.x = pose[0]
        model_state.pose.position.y = pose[1]
        model_state.pose.position.z = 0.5    

        #spawn model
        obstacle_model_xml = self.obstacle_path
        self.spawn_model(model_name, obstacle_model_xml, '', model_state.pose, 'world')
        print("Spawned model: {}".format(model_name))

    def delete_model(self, model_name):
        try:
            resp = self.delete_model_proxy(model_name)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Delete Model service call failed: %s" % e)
            return False
    
    def update_obstacle_positions(self):
        obstacle_poses_world = self.map_grid_to_world()
        self.pending_updates = [("obstacle_{}_{}".format(x, y), (x, y)) for x, y in obstacle_poses_world]

    def update_obstacles(self):
        for model_name, pose in self.pending_updates:
            
            if model_name not in self.existing_obstacle:
                self.set_model(model_name, pose)
                self.existing_obstacle.add(model_name)

        for existing_obstacle in list(self.existing_obstacle):
            if not any(existing_obstacle == name for name, _ in self.pending_updates):
                self.remove_obstacle(existing_obstacle)
                # self.existing_obstacle.remove(existing_obstacle)

    # def update_obstacles(self):
    #     for model_name, new_pose in self.pending_updates:
    #         new_x, new_y = new_pose

    #         close_obstacle = None
    #         for existing_obstacle in self.existing_obstacle:
    #             coords_str = existing_obstacle[len("obstacle_"):] 
    #             existing_x, existing_y = map(float, coords_str.split('_'))

    #             if self.distance((existing_x, existing_y), (new_x, new_y)) < 1.5:
    #                 close_obstacle = existing_obstacle
    #                 break

    #         if close_obstacle:
    #             # if close_obstacle in self.existing_obstacle:
    #             self.remove_obstacle(close_obstacle)

    #         if model_name not in self.existing_obstacle:
    #             self.set_model(model_name, new_pose)
    #             self.existing_obstacle.add(model_name)

    #     for existing_obstacle in list(self.existing_obstacle):
    #         if not any(existing_obstacle == name for name, _ in self.pending_updates):
    #             self.remove_obstacle(existing_obstacle)
                
                
    def distance(self, pose1, pose2):
        return ((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)**0.5

    def timer_callback(self, event):           
        if self.flag == True and self.obstacle_msg is not None:
            obstacle_data = np.array(self.obstacle_msg.data).reshape((self.grid_height, self.grid_width))
            
            self.grid = obstacle_data

            self.update_obstacle_positions()
            self.update_obstacles()

        else:
            self.init_obstacle()   
            print('reset map')

if __name__ == "__main__":
    rospy.init_node("real_to_sim_obstacle")
    real_to_sim_transform = RealtoSimObstacle()
    rospy.spin()