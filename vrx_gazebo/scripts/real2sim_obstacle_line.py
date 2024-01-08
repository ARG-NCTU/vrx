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
        # self.pub_state_to_mapgrid = rospy.Publisher("/reset_map", Bool, queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        self.timer_reset_model_state = rospy.Timer(rospy.Duration(0.02), self.timer_cb_reset)
        self.sub_obstacle = rospy.Subscriber("raw_obstacles", Obstacles, self.obstacle_cb, queue_size=1)
        
        self.ws = rospy.get_param('~ws', '1')
        print('ws:', self.ws)
        self.flag = False
        self.obstacle_name = 'real_obstacle_'
        self.max_num = 0
        # self.existing_obstacle = []
        self.existing_obstacle = [[], []]
        self.init_obstacle()
        self.joy = None
        
    def joy_callback(self, joy):
        if self.joy == None:
            self.joy = joy
            return
        joy_trigger = joy.buttons[4] and not self.joy.buttons[4]

        if joy_trigger:
            self.flag = not self.flag 
        self.joy = joy
            
    def obstacle_cb(self, msg):
        self.obstacle_msg = msg
        
    def init_obstacle(self):
        for i in range (76):
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
        # self.existing_obstacle = []
        self.existing_obstacle = [[], []]
        
    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose     
        self.pub_set_model_state.publish(model_state)

    def check_obstacle(self, obstacle_first, obstacle_last):
        threshold = 4
        sum_x = 0
        right_most = None
        left_most = None
        try: 
            if obstacle_first is not None: 
                for j in range (len(obstacle_first)):
                    if right_most is None:
                        right_most = obstacle_first[j]
                    else:
                        if obstacle_first[j][1] < right_most[1]:
                            right_most = obstacle_first[j]
                    if left_most is None:
                        left_most = obstacle_last[j]
                    else:
                        if obstacle_last[j][1] > left_most[1]:
                            left_most = obstacle_last[j]
                    
                    sum_x = sum_x + obstacle_first[j][0] + obstacle_last[j][0]

                if right_most is not None and left_most is not None:    
                    #compare with existing obstacle
                    mean_x = sum_x / (2 * len(obstacle_first))   
                    left_most[0] = mean_x
                    right_most[0] = mean_x
                    if self.existing_obstacle == [[], []]:
                        self.existing_obstacle[0] = left_most
                        self.existing_obstacle[1] = right_most
                    # left
                    elif (self.existing_obstacle[0][1] + threshold) < left_most[1]:
                        self.existing_obstacle[0] = left_most
                    # right
                    elif (self.existing_obstacle[1][1] - threshold) > right_most[1]:
                        self.existing_obstacle[1] = right_most
                    else:
                        pass
        except ValueError:
            pass
        
    
    def adjust_points_to_average(self):
        x_values = [coord[0] for coord in self.existing_obstacle]
        average_x = sum(x_values) / len(x_values)

        for coord in self.existing_obstacle:
            coord[0] = average_x

    def generate_points(self, first_point, last_point, distance_between_points):
        first_point = np.array(first_point)
        last_point = np.array(last_point)
        direction_vector = last_point - first_point
        total_distance = np.linalg.norm(direction_vector)
        normalized_direction = direction_vector / total_distance
        num_points = int(total_distance / distance_between_points)
        intermediate_points = []
        
        intermediate_points.append(first_point.tolist())
        for i in range(1, num_points):
            intermediate_point = first_point + normalized_direction * (i * distance_between_points)
            if (-6.5 >= intermediate_point[1]) or (intermediate_point[1] >= 6):
                intermediate_points.append(intermediate_point.tolist())
                
        intermediate_points.append(last_point.tolist())
        # print('intermediate_points:', intermediate_points)
        return intermediate_points
    
    def point_to_pose_to_set_model(self, point, order):
        pose = PoseStamped()  
        for i in range (len(point)):
            model_name = self.obstacle_name + str(order + i)
            pose.pose.position.x = point[i][0]
            pose.pose.position.y = point[i][1]
            pose.pose.position.z = 0.5
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            self.set_model(model_name = model_name, pose = pose)
            
    # Function to check if points form a line parallel to y-axis
    def is_parallel_to_x(self, points, threshold):  
        # x_coordinates = [point[0] for point in points]
        return all(abs(x - points[0]) < threshold for x in points)

    # Function to check if points form a line parallel to y-axis
    def is_parallel_to_y(self, points, threshold):
        # y_coordinates = [point[1] for point in points]
        return all(abs(y - points[0]) < threshold for y in points)

          
    def dist(self, p1, p2):

        dis =  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dis   


    def rm_original_obstacles(self, first_point, last_point):
        x1 = first_point[0]
        y1 = first_point[1]
        x2 = last_point[0]
        y2 = last_point[1]
        
        # Define the boundaries of the block
        if self.ws == 1:
            top_boundary = 45
            bottom_boundary = 40
            left_boundary = 25
            right_boundary = -40
    
        elif self.ws == 2:
            top_boundary = 70
            bottom_boundary = 50
            left_boundary = 68
            right_boundary = -68
              
        # filter out wamv3 4
        if self.dist(first_point, last_point) < 7:
            # print('wamv3 4')
            return False
        
        #filter out original obstacles
        elif right_boundary <= y1 <= left_boundary and \
            bottom_boundary <= x1 <= top_boundary and \
            right_boundary <= y2 <= left_boundary and \
            bottom_boundary <= x2 <= top_boundary :
            print('original obstacle')
            return False    
        
        else:
            return True
    
    
    def timer_cb_reset(self, event):
        if self.flag == False:
            self.init_obstacle()   
            print('reset map')
            self.existing_obstacle = [[], []]
            return
        else:
            pass
              
    def timer_callback(self, event):    
        if self.flag == True:
            cnt = 0
            obstacle_first = []
            obstacle_last = []
            for segment in self.obstacle_msg.segments:
                if self.rm_original_obstacles([segment.first_point.x, segment.first_point.y], [segment.last_point.x, segment.last_point.y]):
                    obstacle_first.append([segment.first_point.x, segment.first_point.y])
                    obstacle_last.append([segment.last_point.x, segment.last_point.y])
                else:
                    pass
            
                self.check_obstacle(obstacle_first, obstacle_last)
                order = 0
                self.adjust_points_to_average
                print('existing_obstacle:',self.existing_obstacle)

                if self.existing_obstacle != [[], []]:
                    if (self.dist(self.existing_obstacle[0], self.existing_obstacle[1]) <= 95):
                        points = self.generate_points(self.existing_obstacle[0], self.existing_obstacle[1], 1.5)     
                        self.point_to_pose_to_set_model(points, order)
                        order += len(points)

            else:
                pass
        else:
            pass# print('*************************'       


if __name__ == "__main__":
    rospy.init_node("real_to_sim_obstacle")
    real_to_sim_transform = RealtoSimObstacle()
    rospy.spin()
