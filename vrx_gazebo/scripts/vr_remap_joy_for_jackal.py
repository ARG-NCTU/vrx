#!/usr/bin/env python
from sensor_msgs.msg import Joy
import rospy

class VR_remap_joy:
    # This script remaps the VR teleop to the joy topic
    # For mixed teleop
    def __init__(self):
        self.sub_joy = rospy.Subscriber("/vr_teleop", Joy, self.cb_joy, queue_size=1)
        # self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)      
        self.pub_joy= rospy.Publisher("/jackal/bluetooth_teleop/joy", Joy, queue_size=1)

        self.vr_to_joy = Joy()
        self.vr_to_joy.header.frame_id = "/dev/input/js0"
        self.vr_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vr_to_joy.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.pub_once = True
    
    def cb_joy(self, msg):
        print(msg)
        self.vr_to_joy.header.stamp = rospy.Time.now()
       
        #check robot_id
        self.vr_to_joy.axes[2] = int(msg.axes[1]) # robot
        
        
        self.vr_to_joy.buttons[6] = msg.buttons[0] # Back: manual
        self.vr_to_joy.buttons[7] = msg.buttons[1] # Start: RL
        self.vr_to_joy.buttons[3] = msg.buttons[2] # Y: DP
        
        #wamv DP then jackal switch to manual
        if self.vr_to_joy.buttons[3] == 1 :
            self.vr_to_joy.buttons[3] = 0
            self.vr_to_joy.buttons[6] = 1
            self.vr_to_joy.buttons[7] = 0

        #axes
        self.vr_to_joy.axes[1] = msg.axes[4] # left stick forward/backward
        self.vr_to_joy.axes[0] = msg.axes[5] # right stick right/left
        # self.vr_to_joy.axes[5] = int(msg.axes[0]) # user_id
        
        # if self.vr_to_joy.axes[2] == 2:
        self.pub_joy.publish(self.vr_to_joy)
        

if __name__ == '__main__':
    rospy.init_node('vr_remap_joy_jackal')
    vr_remap_joy = VR_remap_joy()
    # rospy.spin()
    while not rospy.is_shutdown():
        if vr_remap_joy.sub_joy.get_num_connections() == 0:
            # print("No messages received on /vr_teleop yet...")
            pass
    rospy.sleep(0.1) 