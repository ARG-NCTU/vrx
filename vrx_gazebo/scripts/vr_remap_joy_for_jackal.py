#!/usr/bin/env python
from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Bool

class VR_remap_joy:
    # This script remaps the VR teleop to the joy topic
    # For mixed teleop
    def __init__(self):
        self.sub_joy = rospy.Subscriber("/vr_teleop", Joy, self.cb_joy, queue_size=1)
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)  
        self.pub_shutdown_joy_remap_joy = rospy.Publisher("/shutdown_joy_remap_joy", Bool, queue_size=1)
        self.pub_joy_1= rospy.Publisher("/jackal/bluetooth_teleop/joy", Joy, queue_size=1)
        self.pub_joy_2 = rospy.Publisher("/wamv2/joy", Joy, queue_size=1)
        self.pub_joy_3 = rospy.Publisher("/wamv3/joy", Joy, queue_size=1)
        self.pub_joy_4 = rospy.Publisher("/wamv4/joy", Joy, queue_size=1)

        self.vr_to_joy = Joy()
        self.vr_to_joy.header.frame_id = "/dev/input/js0"
        self.vr_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vr_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_once = True
    
    def cb_joy(self, msg):
        if self.pub_once:
            self.pub_shutdown_joy_remap_joy.publish(True)
            # manual jackal
            self.vr_to_joy.axes[2] = 1
            self.vr_to_joy.buttons[4] = 1
            self.pub_joy_1.publish(self.vr_to_joy)

            # DP in th beginning
            self.vr_to_joy.buttons[4] = 0
            self.vr_to_joy.buttons[3] = 1
            self.vr_to_joy.buttons[7] = 1
            #wamv2 DP
            self.vr_to_joy.axes[2] == 2
            self.pub_joy_2.publish(self.vr_to_joy)
            #wamv3 DP
            self.vr_to_joy.axes[2] = 3
            self.pub_joy_3.publish(self.vr_to_joy)
            #wamv4 DP
            self.vr_to_joy.axes[2] = 4
            self.pub_joy_4.publish(self.vr_to_joy)
            
            self.pub_once = False

        self.vr_to_joy.header.stamp = rospy.Time.now()
       
        
        #button 
        self.vr_to_joy.buttons[6] = msg.buttons[0] # Back: manual
        self.vr_to_joy.buttons[7] = msg.buttons[1] # Start: RL
        self.vr_to_joy.buttons[3] = msg.buttons[2] # Y: DP
        self.vr_to_joy.buttons[4] = msg.buttons[3] # reset
        self.vr_to_joy.buttons[2] = msg.buttons[4] # X 
        ## PX4
        self.vr_to_joy.buttons[0] = msg.buttons[5] # A : Arm
        self.vr_to_joy.buttons[1] = msg.buttons[6] # B : offboard
        # print(msg)
        
        #axes
        self.vr_to_joy.axes[1] = msg.axes[4] # left stick forward/backward
        self.vr_to_joy.axes[3] = msg.axes[5] # right stick right/left
        self.vr_to_joy.axes[2] = int(msg.axes[1]) # robot
        self.vr_to_joy.axes[5] = int(msg.axes[0]) # user_id


        # wamv DP then jackal switch to auto
        if self.vr_to_joy.buttons[3] == 1:
            self.vr_to_joy.buttons[7] = 1
        
        if self.vr_to_joy.axes[2] == 2: #robot_id
            self.pub_joy_2.publish(self.vr_to_joy)
            # jackal
            self.vr_to_joy.axes[2] = 1 #robot_id
            self.vr_to_joy.buttons[4] = 1 #trigger button (jackal)
            if self.vr_to_joy.buttons[3] == 1 : # wamv DP then jackal switch to manual
                self.vr_to_joy.buttons[3] = 0
                self.vr_to_joy.buttons[6] = 1
                self.vr_to_joy.buttons[7] = 0
            #axes translate into jackal mode
            self.vr_to_joy.axes[1] = msg.axes[4] # left stick forward/backward
            self.vr_to_joy.axes[0] = msg.axes[5] # right stick right/left
            self.pub_joy_1.publish(self.vr_to_joy)
            
        if self.vr_to_joy.axes[2] == 3: #robot_id
            self.pub_joy_3.publish(self.vr_to_joy)
        if self.vr_to_joy.axes[2] == 4: #robot_id
            self.pub_joy_4.publish(self.vr_to_joy)

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