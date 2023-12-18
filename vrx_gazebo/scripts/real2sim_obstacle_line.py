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
        self.sub_obstacle = rospy.Subscriber("raw_obstacles", Obstacles, self.obstacle_cb, queue_size=1)
        
        self.ws = rospy.get_param('~ws', '1')
        print('ws:', self.ws)
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
        # print('joy_trigger:', joy_trigger)
        
        if joy_trigger:
            self.flag = not self.flag 
            # print('self.flag:', self.flag)       
            if self.flag == True: # reset done
                print('pub true')
                # self.pub_state_to_mapgrid.publish(True)
            else:
                pass
            
        self.joy = joy
            
    def obstacle_cb(self, msg):
        self.obstacle_msg = msg
        
    def init_obstacle(self):
        for i in range (75):
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
        self.existing_obstacle = []
        
    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose     
        self.pub_set_model_state.publish(model_state)
        
    def check_obstacle(self, obstacle_first, obstacle_last):
        threshold = 4
        # print(obstacle_first, obstacle_last)
        try: 
            if not self.existing_obstacle:
                print('first time')
                self.existing_obstacle = [obstacle_first, obstacle_last]
            
            else:
                #compare with existing obstacle
                for j in range (len(obstacle_first)):
                    obstacle_updated = False
                    new_obstacle_length = self.dist(obstacle_first[j], obstacle_last[j])
                              
                    for i in range (len(self.existing_obstacle[0])):
                        existing_obstacle_length = self.dist(self.existing_obstacle[0][i], self.existing_obstacle[1][i])
                        dis_first = self.dist(obstacle_first[j], self.existing_obstacle[0][i])
                        dis_last = self.dist(obstacle_last[j], self.existing_obstacle[1][i])
                        if new_obstacle_length > existing_obstacle_length:
                            if dis_first < threshold and dis_last < threshold: 
                                #new line and existing line have overlap and new line is longer than existing line
                                # update obstacle
                                self.existing_obstacle[0][i] = obstacle_first[j] # new_first 
                                self.existing_obstacle[1][i] = obstacle_last[j] # new_last  
                                obstacle_updated = True
                                break
                            #     # return True 
                            elif dis_first < threshold or dis_last < threshold: # Extended obstacles
                                new_first_old_last = self.dist(obstacle_first[j], self.existing_obstacle[1][i])
                                old_first_new_last = self.dist(self.existing_obstacle[0][i], obstacle_last[j])
                                
                                if new_first_old_last < old_first_new_last:
                                    self.existing_obstacle[1][i] = obstacle_last[j]
                                elif new_first_old_last > old_first_new_last:
                                    # update obstacle
                                    self.existing_obstacle[0][i] = obstacle_first[j]
                                else:
                                    print('bobobobobo')
                                obstacle_updated = True
                                break
                        elif new_obstacle_length < existing_obstacle_length:
                            # new line is shorter than existing line 
                            if dis_first < threshold and dis_last < threshold: 
                                #have overlap 
                                obstacle_updated = True
                                break
                            elif dis_first < threshold or dis_last < threshold: #Extended obstacles
                                new_first_old_last = self.dist(obstacle_first[j], self.existing_obstacle[1][i])
                                old_first_new_last = self.dist(self.existing_obstacle[0][i], obstacle_last[j])                              
                                if new_first_old_last < old_first_new_last:
                                    self.existing_obstacle[1][i] = obstacle_last[j]
                                elif new_first_old_last > old_first_new_last:
                                    # update obstacle
                                    self.existing_obstacle[0][i] = obstacle_first[j]
                                else:
                                    print('bibibibibi')
                                obstacle_updated = True
                                break
                        else:
                            pass  
                    
                    if not obstacle_updated:
                        print('notupdate:', obstacle_first[j], obstacle_last[j])

                    #     self.existing_obstacle[0].append(obstacle_first[j])
                    #     self.existing_obstacle[1].append(obstacle_last[j])
                  
        except ValueError:
            pass
        
    def generate_points(self, first_point, last_point, distance_between_points):
        threshold = 1
        all_points = [first_point] + [last_point]
        # print(all_points)
        mean_x, mean_y = None, None
        
        if self.is_parallel_to_x(all_points, threshold):
            # print('parallel to x')
            x_coordinates = [point[0] for point in all_points]
            mean_x = sum(x_coordinates) / len(x_coordinates)    
            # print('mean x:', mean_x)
            
        if self.is_parallel_to_y(all_points, threshold):
            # print('parallel to y')
            y_coordinates = [point[1] for point in all_points]
            mean_y = sum(y_coordinates) / len(y_coordinates)    
            # print('mean y:', mean_y)
        
        if mean_x != None:
            first_point[0] = mean_x
            last_point[0] = mean_x  
        if mean_y != None:
            first_point[1] = mean_y
            last_point[1] = mean_y
        
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
            intermediate_points.append(intermediate_point.tolist())
        intermediate_points.append(last_point.tolist())
        # print(intermediate_points)
        
        return intermediate_points
    
    def point_to_pose_to_set_model(self, point, order):
        pose = PoseStamped()  
        # print('****************')
        # print(len(point))   
        # print('****************')
        
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
        x_coordinates = [point[0] for point in points]
        return all(abs(x - x_coordinates[0]) < threshold for x in x_coordinates)

    # Function to check if points form a line parallel to y-axis
    def is_parallel_to_y(self, points, threshold):
        y_coordinates = [point[1] for point in points]
        return all(abs(y - y_coordinates[0]) < threshold for y in y_coordinates)

          
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
            self.existing_obstacle = []
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
                
            # print('*************************')
            # print('obstacle_first:', obstacle_first)
            # print('obstacle_last:', obstacle_last)
            
            self.check_obstacle(obstacle_first, obstacle_last)
            order = 0
            # print('existing_obstacle:',self.existing_obstacle)
            for i in range (len(self.existing_obstacle[0])):
                points = self.generate_points(self.existing_obstacle[0][i], self.existing_obstacle[1][i], 1.5)     
                self.point_to_pose_to_set_model(points, order)
                order += len(points)
                # print('order:', order)
        else:
            pass# print('*************************'       


if __name__ == "__main__":
    rospy.init_node("real_to_sim_obstacle")
    real_to_sim_transform = RealtoSimObstacle()
    rospy.spin()
