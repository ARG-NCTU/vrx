#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool
import tf.transformations as tf_trans
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from pyexpat import model
from sensor_msgs.msg import Joy
from obstacle_detector.msg import Obstacles
import math 
import tf2_ros


class InitWamvPose:
    def __init__(self):
        self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.sub_wamv = rospy.Subscriber('/gazebo/wamv/pose', PoseStamped, self.wamv_cb)
        self.timer_cb = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.cnt = 0

    def init_pose(self, model, x, y, z=-0.090229, ori_x=0, ori_y=0, ori_z=0, ori_w=1):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = ori_x
        pose.pose.orientation.y = ori_y
        pose.pose.orientation.z = ori_z
        pose.pose.orientation.w = ori_w
        self.set_model(model, pose)

    def wamv_cb(self,msg):
        # print(msg)
        self.wamv_pose = msg.pose.position
        self.wamv_rot_x = msg.pose.orientation.x
        self.wamv_rot_y = msg.pose.orientation.y
        self.wamv_rot_z = msg.pose.orientation.z
        self.wamv_rot_w = msg.pose.orientation.w

    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose
        print('model set:', model_name)
        self.pub_set_model_state.publish(model_state)     
    
    def timer_callback(self,event):
        if self.cnt < 5 :
            self.init_pose(model='wamv3',x=10,y=50)
            self.init_pose(model='wamv4',x=10,y=-50)
            self.init_pose(model='wamv2',x=10,y=0,z=-0.090229,ori_x = self.wamv_rot_x, ori_y=self.wamv_rot_y, ori_z=self.wamv_rot_z, ori_w=self.wamv_rot_w) 
            self.cnt+=1
        else:
            rospy.signal_shutdown("Pose Set, Shutting down.")  # Shuts down the node
            
            
if __name__ == "__main__":
    rospy.init_node("init_Wamv_pose")
    init_Wamv_pose = InitWamvPose()
    rospy.spin()
