#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped

class InitWamvPose:
    def __init__(self):
        self.pub_set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.timer_cb = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.wamv2_init = rospy.get_param("~wamv2_init", [10, 0])
        self.wamv3_init = rospy.get_param("~wamv3_init", [10, 50])
        self.wamv4_init = rospy.get_param("~wamv4_init", [10, -50])
        self.cnt = 0

        self.wamv2_init = [int(x) for x in self.wamv2_init.split(",")]
        self.wamv3_init = [int(x) for x in self.wamv3_init.split(",")]
        self.wamv4_init = [int(x) for x in self.wamv4_init.split(",")]
        
    def init_pose(self, model, x, y, z=-0.090229, ori_x=0, ori_y=0, ori_z=0.707, ori_w=0.707):

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = ori_x
        pose.pose.orientation.y = ori_y
        pose.pose.orientation.z = ori_z
        pose.pose.orientation.w = ori_w
        self.set_model(model, pose)

        
    def set_model(self, model_name, pose):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose = pose.pose
        print('model set:', model_name)
        self.pub_set_model_state.publish(model_state)     
    
    def timer_callback(self,event):

        if self.cnt < 8:
            self.init_pose(model='wamv3',x=self.wamv3_init[0], y=self.wamv3_init[1])
            self.init_pose(model='wamv4',x=self.wamv4_init[0], y=self.wamv4_init[1])
            self.init_pose(model='wamv2',x=self.wamv2_init[0], y=self.wamv2_init[1]) 
            self.cnt+=1
            print('wamv2 pose set to:',self.wamv2_init)
            print('wamv3 pose set to:',self.wamv3_init)
            print('wamv4 pose set to:',self.wamv4_init)
        else:
            rospy.signal_shutdown("Pose Set, Shutting down.")  # Shuts down the node
                
                
if __name__ == "__main__":
    rospy.init_node("init_Wamv_pose")
    init_Wamv_pose = InitWamvPose()
    rospy.spin()
