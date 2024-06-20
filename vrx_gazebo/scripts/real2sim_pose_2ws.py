#!/usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tf_trans
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf2_ros
import copy

class RealtoSimTransform:
    def __init__(self):
        # self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.pub_pose = rospy.Publisher("/fake_fence_real2sim", PoseStamped, queue_size=1)
        self.pub_pose_bigwave = rospy.Publisher("/fake_fence_real2sim_bigwave", PoseStamped, queue_size=1)
        self.sub_wamv_pose = rospy.Subscriber("real_pose", PoseStamped, self.wamv_pose_callback)
        self.scale_transform = rospy.get_param("scale_transform", 1)

        self.sub_wamv2_pose = rospy.Subscriber("sim_pose", PoseStamped, self.wamv2_pose_callback)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.sync_freq = rospy.get_param("sync_freq", 20)
        self.timer_set_model_state = rospy.Timer(rospy.Duration(0.05), self.timer_callback)   
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        
        self.flag = False
        self.matrix_wamv_origin_to_map = None
        self.matrix_wamv2_origin_to_map = None
        self.wamv_pose = PoseStamped()
        self.wamv2_pose = PoseStamped()

        # self.init_wamv2 = PoseStamped()        
        self.time = rospy.Time.now()
        self.joy = None

    def wamv_pose_callback(self, pose):
        self.wamv_pose = pose

    def wamv2_pose_callback(self, pose):
        self.wamv2_pose = pose
        
    def joy_callback(self, joy):

        if self.joy is None:
            self.joy = joy
            return

        # joy_trigger = joy.buttons[4] and not self.joy.buttons[4] 
        joy_trigger = joy.buttons[0] and not self.joy.buttons[0] 
    
        if joy_trigger:
            print('start sync')   
            self.matrix_wamv_origin_to_map = self.pose_to_matrix(self.wamv_pose)
            self.matrix_wamv2_origin_to_map = self.pose_to_matrix(self.wamv2_pose)
            self.flag = True
        else:
            pass
        self.joy = joy

    def timer_callback(self, event):
        if self.flag == True: 
            # transform
            if self.matrix_wamv_origin_to_map is None or self.matrix_wamv2_origin_to_map is None:
                return
            matrix_wamv_to_map = self.pose_to_matrix(self.wamv_pose)
            inv_mat_wamv_origin_to_map = tf_trans.inverse_matrix(self.matrix_wamv_origin_to_map)
            matrix_wamv_to_origin = np.dot(inv_mat_wamv_origin_to_map, matrix_wamv_to_map)

            matrix_wamv_to_origin[0][3] *= self.scale_transform
            matrix_wamv_to_origin[1][3] *= self.scale_transform

            matrix_wamv2_to_map = np.dot(self.matrix_wamv2_origin_to_map, matrix_wamv_to_origin)
            # pub
            pose_wamv2_to_map = self.matrix_to_pose(matrix_wamv2_to_map, "map")
            pose_wamv2_to_map.pose.position.z = -0.090229
            euler = tf_trans.euler_from_quaternion(
                [
                    pose_wamv2_to_map.pose.orientation.x,
                    pose_wamv2_to_map.pose.orientation.y,
                    pose_wamv2_to_map.pose.orientation.z,
                    pose_wamv2_to_map.pose.orientation.w,
                ]
            )
            
            q = tf_trans.quaternion_from_euler(0, 0, euler[2])
            pose_wamv2_to_map.pose.orientation.x = q[0] #self.wamv_pose.pose.orientation.x 
            pose_wamv2_to_map.pose.orientation.y = q[1] #self.wamv_pose.pose.orientation.y 
            pose_wamv2_to_map.pose.orientation.z = q[2]
            pose_wamv2_to_map.pose.orientation.w = q[3]
            
            # q_bigwave = tf_trans.quaternion_from_euler(euler[0], euler[1], euler[2])
            pose_wamv2_to_map_bigwave = copy.deepcopy(pose_wamv2_to_map)
            pose_wamv2_to_map_bigwave.pose.orientation.x = self.wamv_pose.pose.orientation.x 
            pose_wamv2_to_map_bigwave.pose.orientation.y = self.wamv_pose.pose.orientation.y
            # pose_wamv2_to_map_bigwave.pose.orientation.x = q_bigwave[0]
            # pose_wamv2_to_map_bigwave.pose.orientation.y = q_bigwave[1]
            # pose_wamv2_to_map_bigwave.pose.orientation.z = q_bigwave[2]
            # pose_wamv2_to_map_bigwave.pose.orientation.w = q_bigwave[3]
            
            print(pose_wamv2_to_map.pose.orientation)
            self.tf_msg.header.stamp = rospy.Time.now()
            self.tf_msg.header.frame_id = "map"
            self.tf_msg.child_frame_id = "wamv2/base_link"
            self.tf_msg.transform.translation = pose_wamv2_to_map.pose.position
            self.tf_msg.transform.rotation = pose_wamv2_to_map.pose.orientation
            self.tf_broadcaster.sendTransform(self.tf_msg)
            
            self.pub_pose.publish(pose_wamv2_to_map)
            self.pub_pose_bigwave.publish(pose_wamv2_to_map_bigwave)

            print('pose:', pose_wamv2_to_map)
            self.set_model(model_name = "wamv2", pose = pose_wamv2_to_map)    
            
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
