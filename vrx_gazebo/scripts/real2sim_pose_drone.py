#!/usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf2_ros

class RealtoSimTransformDrone:
    def __init__(self):
        self.pub_pose = rospy.Publisher("/drone_pose", PoseStamped, queue_size=1)
        self.sub_wamv_pose = rospy.Subscriber("/gazebo/wamv/pose", PoseStamped, self.wamv_pose_callback)
        self.sub_wamv2_pose = rospy.Subscriber("/gazebo/wamv2/pose", PoseStamped, self.wamv2_pose_callback)
        self.sub_drone_pose = rospy.Subscriber("/gazebo/drone/pose", PoseStamped, self.drone_pose_callback)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.sync_freq = rospy.get_param("sync_freq", 20)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.05), self.timer_callback)   
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        
        self.flag = False
        self.matrix_wamv_origin_to_map = None
        self.matrix_drone_origin_to_map = None
        self.wamv_pose = PoseStamped()
        self.drone_pose = PoseStamped()
  
        self.time = rospy.Time.now()
        self.joy = None

    def wamv_pose_callback(self, pose):
        self.wamv_pose = pose
        
    def wamv2_pose_callback(self, pose):
        self.wamv2_pose = pose
        
    def drone_pose_callback(self, pose):
        self.drone_pose = pose
        
    def joy_callback(self, joy):

        if self.joy is None:
            self.joy = joy
            return

        joy_trigger = joy.buttons[4] and not self.joy.buttons[4] 
    
        if joy_trigger:
            print('start sync')               
            self.matrix_wamv_origin_to_map = self.pose_to_matrix(self.wamv_pose)
            self.matrix_wamv2_origin_to_map = self.pose_to_matrix(self.wamv2_pose)
            self.matrix_drone_origin_to_map = self.pose_to_matrix(self.drone_pose)
            
            self.flag = True
        else:
            pass
        
        self.joy = joy

    def timer_callback(self, event):
        if self.flag == True: 
            # transform
            if self.wamv2_pose is None or self.drone_pose is None or self.wamv_pose is None:
                print('No pose')
                return
            matrix_wamv_to_map = self.pose_to_matrix(self.wamv_pose)
            matrix_drone_to_map = self.pose_to_matrix(self.drone_pose)
            
            inv_matrix_drone_to_map = tf_trans.inverse_matrix(matrix_drone_to_map)
            matrix_wamv_to_drone = np.dot(inv_matrix_drone_to_map, matrix_wamv_to_map)
            
            # inv_matrix_drone_to_wamv = tf_trans.inverse_matrix(matrix_drone_to_wamv)
            matrix_wamv2_to_map = self.pose_to_matrix(self.wamv2_pose)
            inv_matrix_wamv2_to_map = tf_trans.inverse_matrix(matrix_wamv2_to_map)
            inv_matrix_drone_to_map = np.dot(matrix_wamv_to_drone, inv_matrix_wamv2_to_map)
            matrix_drone_to_map = tf_trans.inverse_matrix(inv_matrix_drone_to_map)
            
            #pub
            pose_drone_to_map = self.matrix_to_pose(matrix_drone_to_map, "map")
            
            # print(matrix_drone_to_map)
            
            euler = tf_trans.euler_from_quaternion(
                [
                    pose_drone_to_map.pose.orientation.x,
                    pose_drone_to_map.pose.orientation.y,
                    pose_drone_to_map.pose.orientation.z,
                    pose_drone_to_map.pose.orientation.w,
                ]
            )
            # q = tf_trans.quaternion_from_euler(0, 0, euler[2])
            q = tf_trans.quaternion_from_euler(euler[0], euler[1], euler[2])
            pose_drone_to_map.pose.orientation.x = q[0] 
            pose_drone_to_map.pose.orientation.y = q[1] 
            pose_drone_to_map.pose.orientation.z = q[2]
            pose_drone_to_map.pose.orientation.w = q[3]
                
            self.pub_pose.publish(pose_drone_to_map)
            print('pose:', pose_drone_to_map) 
            

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
    rospy.init_node("drone_pose_transform")
    real_to_sim_transform_drone = RealtoSimTransformDrone()
    rospy.spin()
