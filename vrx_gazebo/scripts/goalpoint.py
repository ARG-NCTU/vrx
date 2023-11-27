#!/usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tf_trans
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from pyexpat import model
from sensor_msgs.msg import Joy
from obstacle_detector.msg import Obstacles
import math 

class RealtoSimTransform:
    def __init__(self):
        # self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.pub_pose = rospy.Publisher("/fake_fence_real2sim", PoseStamped, queue_size=1)
        self.sub_wamv_pose = rospy.Subscriber("/wamv/truth_map_posestamped", PoseStamped, self.wamv_pose_callback)
        self.sub_wamv2_pose = rospy.Subscriber("/wamv2/truth_map_posestamped", PoseStamped, self.wamv2_pose_callback)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        self.sub_obstacle = rospy.Subscriber('/raw_obstacles', Obstacles, self.obstacle_cb, queue_size=1)

        self.matrix_wamv_origin_to_map = None
        self.matrix_wamv2_origin_to_map = None
        self.wamv_pose = PoseStamped()
        self.wamv2_pose = PoseStamped()

        self.start_x = rospy.get_param("~x", 30)
        self.start_y = rospy.get_param("~y", 30)
        self.start_z = rospy.get_param("~z",  -0.090229)
        
        self.init_pose = PoseStamped()
        self.init_pose.pose.position.x = self.start_x
        self.init_pose.pose.position.y = self.start_y
        self.init_pose.pose.position.z = self.start_z
        
        self.flag = False
        self.obstacle_name = 'real_obstacle_'
        self.max_num = 0
        self.existing_obstacle = []

    #     self.sub_scan = rospy.Subscriber("/wamv/RL/more_scan", LaserScan, self.cb_laser_2)
    #     self.pub_scan_2 = rospy.Publisher("/wamv/RL/more_scan_2", LaserScan, queue_size=1)

    # def cb_laser_2(self, msg):
    #     laser = msg
    #     laser.header.stamp = rospy.Time.now()
    #     laser.header.frame_id = "wamv2/lidar_wamv_link"
    #     self.pub_scan_2.publish(laser)

    def wamv_pose_callback(self, pose):
        self.wamv_pose = pose

    def wamv2_pose_callback(self, pose):
        self.wamv2_pose = pose

    def joy_callback(self, joy):
        self.joy = joy
        joy_trigger = self.joy.buttons[4] or self.joy.buttons[5] #LB /B
 
        if joy_trigger and self.flag ==False:
            print('start sync')
            self.matrix_wamv_origin_to_map = self.pose_to_matrix(self.wamv_pose)
            self.matrix_wamv2_origin_to_map = self.pose_to_matrix(self.wamv2_pose)
            self.flag = True
            print(self.flag)

        elif joy_trigger:
            #set wamv2 pose
            self.init_pose.pose.orientation = self.wamv_pose.pose.orientation
            self.init_pose.pose.orientation.x = 0
            self.init_pose.pose.orientation.y = 0
            self.set_model(model_name = "wamv2", pose = self.init_pose)
            self.flag = False
            print('stop sync')
            
        
    def obstacle_cb(self, msg):
        cnt = 0
        for circle in msg.circles:
            obstacle_pose = PoseStamped()
            obstacle_pose.pose.position.x = circle.center.x
            obstacle_pose.pose.position.y = circle.center.y
            obstacle_pose.pose.position.z = circle.center.z
            obstacle_pose.pose.orientation.x = 0
            obstacle_pose.pose.orientation.y = 0
            obstacle_pose.pose.orientation.z = 0
            obstacle_pose.pose.orientation.w = 1
            model_name = self.obstacle_name + str(cnt)
            new_model = {'model': model_name, 
                        'pose_x': obstacle_pose.pose.position.x, 
                        'pose_y': obstacle_pose.pose.position.y}
            update_name = self.check_obstacle(new_model) 
            if self.flag == True:
                if  update_name == False:
                    self.set_model(model_name = model_name, pose = obstacle_pose)
                    self.existing_obstacle.append(new_model)
                    print('new obstacle:', model_name)
                else:
                    self.set_model(model_name = update_name, pose = obstacle_pose)
                    print('update obstacle:', update_name)
                    
            else:
                self.existing_obstacle = []
                self.init_obstacle()
                # print('no obstacle detect')
                pass
            cnt += 1

    def init_obstacle(self):
        # print('inint_obstacle')
        for i in range (25):
            obstacle_pose = PoseStamped()
            model_name = self.obstacle_name + str(i)
            obstacle_pose.pose.position.x = 60
            obstacle_pose.pose.position.y = i + 2
            obstacle_pose.pose.position.z = -50 
            obstacle_pose.pose.orientation.x = 0
            obstacle_pose.pose.orientation.y = 0
            obstacle_pose.pose.orientation.z = 0
            obstacle_pose.pose.orientation.w = 1
            self.set_model(model_name = model_name, pose = obstacle_pose)

    # def model_callback(self, msg):
    #     self.existing_obstacle_x = []
    #     self.existing_obstacle_y = []
    #     try:
    #         for i in range (self.max_num):
    #             model = self.obstacle_name + str(i)
    #             model_index = msg.name.index(model)
    #             self.existing_obstacle_x.append(msg.pose[model_index].position.x)
    #             self.existing_obstacle_y.append(msg.pose[model_index].position.y)          
    #             print('obstacle_x:', self.existing_obstacle_x)   
    #             # print(len(self.existing_obstacle_x))
    #     except ValueError:
    #         pass
        
    def check_obstacle(self, new_model):
        try:
            if self.existing_obstacle == None:
                self.existing_obstacle.append(new_model)
                return False
            else:
                for i in range(len(self.existing_obstacle)):
                    dis = self.dist([self.existing_obstacle[i]['pose_x'], self.existing_obstacle[i]['pose_y']], [new_model['pose_x'], new_model['pose_y']])
                    if dis < 3 : #update obstacle
                        self.existing_obstacle[i]['pose_x'] = new_model['pose_x']
                        self.existing_obstacle[i]['pose_y'] = new_model['pose_y']
                        model_name = self.obstacle_name + str(i) 
                        return model_name
                    else:
                        # self.existing_obstacle.append(new_model)
                        pass
                return False
            
        except ValueError:
            pass
            
    def dist(self, p1, p2):
        dis =  math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return dis   

    def timer_callback(self, event):
        
        if self.flag == True: 
            # transform
            if self.matrix_wamv_origin_to_map is None or self.matrix_wamv2_origin_to_map is None:
                return
            matrix_wamv_to_map = self.pose_to_matrix(self.wamv_pose)
            inv_mat_wamv_origin_to_map = tf_trans.inverse_matrix(self.matrix_wamv_origin_to_map)
            matrix_wamv_to_origin = np.dot(inv_mat_wamv_origin_to_map, matrix_wamv_to_map)

            matrix_wamv2_to_map = np.dot(self.matrix_wamv2_origin_to_map, matrix_wamv_to_origin)
            # pub
            pose_wamv2_to_map = self.matrix_to_pose(matrix_wamv2_to_map, "map")
            pose_wamv2_to_map.pose.position.z = self.start_z
            euler = tf_trans.euler_from_quaternion(
                [
                    pose_wamv2_to_map.pose.orientation.x,
                    pose_wamv2_to_map.pose.orientation.y,
                    pose_wamv2_to_map.pose.orientation.z,
                    pose_wamv2_to_map.pose.orientation.w,
                ]
            )
            q = tf_trans.quaternion_from_euler(0, 0, euler[2])
            pose_wamv2_to_map.pose.orientation.x = q[0]
            pose_wamv2_to_map.pose.orientation.y = q[1]
            pose_wamv2_to_map.pose.orientation.z = q[2]
            pose_wamv2_to_map.pose.orientation.w = q[3]
            model_state = ModelState()
            model_state.model_name = "wamv2"
            model_state.reference_frame = "world"
            model_state.pose = pose_wamv2_to_map.pose
            self.set_model(model_name = "wamv2", pose = pose_wamv2_to_map)
            self.pub_pose.publish(pose_wamv2_to_map)
            # print("wamv2 pose to map: ", pose_wamv2_to_map)

            # matrix_wamv2_to_origin = np.dot(tf_trans.inverse_matrix(self.matrix_wamv2_origin_to_map), matrix_wamv2_to_map)
            # pose_wamv2_to_origin = self.matrix_to_pose(matrix_wamv2_to_origin, "wamv2")
            # print("wamv2 pose to origin: ", pose_wamv2_to_origin)
            # print("wamv2 pose to map: ", pose_wamv2_to_map)
            
        else:
            pass
        
    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose
        self.pub_set_model_state.publish(model_state)
        
    def pose_to_matrix(self, pose):
        if isinstance(pose, PoseStamped):
            translation = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            rotation = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
            return tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(translation), tf_trans.quaternion_matrix(rotation)
            )

    def matrix_to_pose(self, matrix, frame_id="map"):
        ret_pose_stamped = PoseStamped()
        trans = tf_trans.translation_from_matrix(matrix)
        rot = tf_trans.quaternion_from_matrix(matrix)
        ret_pose_stamped.header.frame_id = frame_id
        ret_pose_stamped.header.stamp = rospy.Time.now()

        ret_pose_stamped.pose.position.x = trans[0]
        ret_pose_stamped.pose.position.y = trans[1]
        ret_pose_stamped.pose.position.z = trans[2]
        ret_pose_stamped.pose.orientation.x = rot[0]
        ret_pose_stamped.pose.orientation.y = rot[1]
        ret_pose_stamped.pose.orientation.z = rot[2]
        ret_pose_stamped.pose.orientation.w = rot[3]
        return ret_pose_stamped


if __name__ == "__main__":
    rospy.init_node("real_to_sim_transform")
    real_to_sim_transform = RealtoSimTransform()
    rospy.spin()
