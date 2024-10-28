#!/usr/bin/env python
from sensor_msgs.msg import Joy
import rospy
class Multi_vr_remap_joy:
    # This script remaps the VR teleop to the joy topic
    # For mixed teleop
    def __init__(self):
        self.sub_vr_joy_1 = rospy.Subscriber("/vr_teleop_1", Joy, self.cb_vr_joy_1, queue_size=1)
        self.sub_vr_joy_2 = rospy.Subscriber("/vr_teleop_2", Joy, self.cb_vr_joy_2, queue_size=1)
        self.sub_vr_joy_3 = rospy.Subscriber("/vr_teleop_3", Joy, self.cb_vr_joy_3, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)
        pub_joy_topic = rospy.get_param("~pub_joy_topic", "/wamv2/joy")
        self.pub_joy = rospy.Publisher(pub_joy_topic, Joy, queue_size=1)
        
        self.vr_joy = None
        self.vr_to_joy = Joy()
        self.vr_to_joy.header.frame_id = "/dev/input/js1"
        self.vr_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vr_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.vr_id = 0 # 1, 2, 3
            
           
    def cb_vr_joy_1(self, msg):
        if self.vr_id != 0:
            self.vr_joy = msg
            self.vr_translate_into_joy()

    def cb_vr_joy_2(self, msg):
        if self.vr_id != 0:
            self.vr_joy = msg
            self.vr_translate_into_joy()

    def cb_vr_joy_3(self, msg):
        if self.vr_id != 0:
            self.vr_joy = msg
            self.vr_translate_into_joy()

    def cb_joy(self, msg):
        # A button: 0 (joystick), X button: 1 (VR joy1), Y button: 2 (VR joy2), B button: 3 (VR joy3)
        if msg.buttons[0] == 1: # A button
            self.vr_id = 0
            rospy.loginfo("Joystick")
        else:
            if msg.buttons[2] == 1: # X button
                self.vr_id = 1
            elif msg.buttons[3] == 1: # Y button
                self.vr_id = 2
            elif msg.buttons[1] == 1: # B button
                self.vr_id = 3
            rospy.loginfo("VR ID: %d", self.vr_id)
        
        if self.vr_id == 0:
            self.pub_joy.publish(msg)

    def vr_translate_into_joy(self):
        #axes
        self.vr_to_joy.axes[1] = self.vr_joy.axes[6] # left stick forward/backward
        self.vr_to_joy.axes[3] = self.vr_joy.axes[9] # right stick right/left
        # self.vr_to_joy.axes[2] = int(self.vr_joy.axes[1]) # robot
        # self.vr_to_joy.axes[5] = int(self.vr_joy.axes[0]) # user_id
        self.vr_to_joy.buttons[4] = 1
        self.vr_to_joy.buttons[6] = 1


        self.pub_joy.publish(self.vr_to_joy)
    
    
if __name__ == '__main__':
    rospy.init_node('multi_vr_remap_joy')
    vr_remap_joy = Multi_vr_remap_joy()
    rospy.spin()
    rospy.sleep(0.1) 