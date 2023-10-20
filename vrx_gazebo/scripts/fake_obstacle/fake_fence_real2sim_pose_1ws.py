#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest


class ShiftPose(object):
    def __init__(self):
        # self.broadcaster = tf.TransformBroadcaster()
        self.fram_name = rospy.get_param("~frame_name", "wamv/base_link")
        self.paraent_name = rospy.get_param("~parent_name", "map")
        
        self.pub_pose = rospy.Publisher("/fake_fence_real2sim", PoseStamped, queue_size=1)
        
        self.pub_odometry = rospy.Publisher('/fake_map_odometry', Odometry, queue_size=1)
        
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)
        self.model = GetModelStateRequest()
        
        self.update_pose = True
        
        #init wamv pose
        self.wamv_x, self.wamv_y, self.wamv_z = 0, 0, 0
        self.wamv_qx, self.wamv_qy, self.wamv_qz, self.wamv_qw = 0, 0, 0, 0
         
        self.wamv2_x, self.wamv2_y, self.wamv2_z = 0, 0, 0
        self.wamv2_qx, self.wamv2_qy, self.wamv2_qz, self.wamv2_qw = 0, 0, 0, 0
        
        self.set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.objstate = SetModelStateRequest()
        self.flag = False
        
    def cb_joy(self, msg):
        start_btn = 7  
        end_btn = 6      
        if (msg.buttons[start_btn] == 1 ) and self.update_pose:
            self.update_pose = False
            self.get_wamv_pose_once(model_name='wamv')
            self.set_wamv_pose(model_name='wamv2', initial=True)
            self.get_wamv_pose_once(model_name='wamv2')
            rospy.loginfo('get wamv pose as origin') 
            self.flag = True
        if(msg.buttons[end_btn] == 1 ) and not self.update_pose:
            self.update_pose = True
            # rospy.loginfo('stop recieve wamv pose as origin')
   
    def set_wamv_pose(self, model_name='wamv2',initial=False, x=0 , y=0, z=0, qx=0, qy=0, qz=0, qw=0):
        
        self.objstate.model_state.model_name = model_name
        
        if initial == True:
            x = rospy.get_param("~x", 0)
            y = rospy.get_param("~y", 50)
            z = rospy.get_param("~z", -0.090229)
            qx = self.wamv_qx
            qy = self.wamv_qy
            qz = self.wamv_qz
            qw = self.wamv_qw
        else:
            pass
        
        # self.objstate.model_state.reference_frame = 'world'
        self.objstate.model_state.pose.position.x = x
        self.objstate.model_state.pose.position.y = y
        self.objstate.model_state.pose.position.z = z
        
        self.objstate.model_state.pose.orientation.x = qx
        self.objstate.model_state.pose.orientation.y = qy
        self.objstate.model_state.pose.orientation.z = qz
        self.objstate.model_state.pose.orientation.w = qw
        
        self.objstate.model_state.twist.linear.x = 0
        self.objstate.model_state.twist.linear.y = 0
        self.objstate.model_state.twist.linear.z = 0
        self.objstate.model_state.twist.angular.x = 0
        self.objstate.model_state.twist.angular.y = 0
        self.objstate.model_state.twist.angular.z = 0
        self.set_model(self.objstate)
        # rospy.loginfo('wamv2 is ready')
        
    def get_wamv_pose_once(self, model_name='wamv'):
        try:
            agent_wamv = self.get_model(model_name, '')
        except (rospy.ServiceException) as e:
            print(e)  
        if model_name == 'wamv':
            self.wamv_x = agent_wamv.pose.position.x
            self.wamv_y = agent_wamv.pose.position.y
            self.wamv_z = agent_wamv.pose.position.z
            self.wamv_qx = agent_wamv.pose.orientation.x
            self.wamv_qy = agent_wamv.pose.orientation.y
            self.wamv_qz = agent_wamv.pose.orientation.z
            self.wamv_qw = agent_wamv.pose.orientation.w
            rospy.loginfo('%s pose x / y = %.2f / %.2f', model_name, self.wamv_x, self.wamv_y)
        elif model_name == 'wamv2':
            self.wamv2_x = agent_wamv.pose.position.x
            self.wamv2_y = agent_wamv.pose.position.y
            self.wamv2_z = agent_wamv.pose.position.z
            self.wamv2_qx = agent_wamv.pose.orientation.x
            self.wamv2_qy = agent_wamv.pose.orientation.y
            self.wamv2_qz = agent_wamv.pose.orientation.z
            self.wamv2_qw = agent_wamv.pose.orientation.w
            rospy.loginfo('%s pose x / y = %.2f / %.2f', model_name, self.wamv2_x, self.wamv2_y)
            
        else:
            pass
        

    def shift_wamv_pose(self, init1, init2, pos1,delta):
        #pos1: wamv pose (real world)
        #pos2: wamv2 pose (fake)
        if init2 >= init1:
            result = pos1 + delta
            return result
        if init2 < init1:
            result = pos1 - delta
            return result
        else:
            pass
        
    def gazebo_odom(self):
        
        agent_wamv= ModelState()
        agent_wamv2= ModelState()
        rospy.wait_for_service('/gazebo/get_model_state')
        
        try:
            agent_wamv = self.get_model('wamv', '')
            agent_wamv2 = self.get_model('wamv2', '')
        except (rospy.ServiceException) as e:
            print(e)
            
        if self.flag == True:
            delta_x = abs(self.wamv_x - self.wamv2_x)
            delta_y = abs(self.wamv_y - self.wamv2_y)
            delta_z = abs(self.wamv_z - self.wamv2_z)
            delta_qx = abs(self.wamv_qx - self.wamv2_qx)
            delta_qy = abs(self.wamv_qy - self.wamv2_qy)
            delta_qz = abs(self.wamv_qz - self.wamv2_qz)
            delta_qw = abs(self.wamv_qw - self.wamv2_qw)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.paraent_name
            pose_msg.pose.position.x = self.shift_wamv_pose(self.wamv_x, self.wamv2_x, agent_wamv.pose.position.x, delta_x)
            pose_msg.pose.position.y = self.shift_wamv_pose(self.wamv_y, self.wamv2_y, agent_wamv.pose.position.y, delta_y)
            pose_msg.pose.position.z = self.shift_wamv_pose(self.wamv_z, self.wamv2_z, agent_wamv.pose.position.z, delta_z)
            pose_msg.pose.orientation.x = self.shift_wamv_pose(self.wamv_qx, self.wamv2_qx, agent_wamv.pose.orientation.x, delta_qx)
            pose_msg.pose.orientation.y = self.shift_wamv_pose(self.wamv_qy, self.wamv2_qy, agent_wamv.pose.orientation.y, delta_qy)
            pose_msg.pose.orientation.z = self.shift_wamv_pose(self.wamv_qz, self.wamv2_qz, agent_wamv.pose.orientation.z, delta_qz)
            pose_msg.pose.orientation.w = self.shift_wamv_pose(self.wamv_qw, self.wamv2_qw, agent_wamv.pose.orientation.w, delta_qw)

            self.pub_pose.publish(pose_msg)

            odom= Odometry()
            odom.header.stamp= rospy.Time.now()
            odom.header.frame_id = "map"
            odom.pose.pose.position.x = pose_msg.pose.position.x
            odom.pose.pose.position.y = pose_msg.pose.position.y
            odom.pose.pose.position.z = pose_msg.pose.position.z
            odom.pose.pose.orientation.x = pose_msg.pose.orientation.x
            odom.pose.pose.orientation.y = pose_msg.pose.orientation.y
            odom.pose.pose.orientation.z = pose_msg.pose.orientation.z
            odom.pose.pose.orientation.w = pose_msg.pose.orientation.w

            self.pub_odometry.publish(odom)

            # make sure wamv fake can be the same pose as wamv real
            self.set_wamv_pose(model_name='wamv2',initial = False, 
                                   x = pose_msg.pose.position.x, 
                                   y = pose_msg.pose.position.y, 
                                   z = pose_msg.pose.position.z, 
                                   qx = pose_msg.pose.orientation.x, 
                                   qy = pose_msg.pose.orientation.y, 
                                   qz = pose_msg.pose.orientation.z, 
                                   qw = pose_msg.pose.orientation.w)

            #     print('x/ y=',pose_msg.pose.position.x, pose_msg.pose.position.y)

if __name__ == "__main__":
    rospy.init_node("shift_pose")
    shift_pose = ShiftPose()
    cnt = 0
    while not rospy.is_shutdown():
        shift_pose.gazebo_odom()
        rospy.sleep(0.1)
