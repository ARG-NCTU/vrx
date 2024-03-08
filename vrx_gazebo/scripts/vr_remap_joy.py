#!/usr/bin/env python3
import fix_python3_path
from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Bool, UInt8MultiArray , UInt8
class VR_remap_joy:
    # This script remaps the VR teleop to the joy topic
    # For mixed teleop
    def __init__(self):
        self.pub_shutdown_joy_remap_joy = rospy.Publisher("/shutdown_joy_remap_joy", Bool, queue_size=1)
        # self.sub_joy = rospy.Subscriber("/vr_teleop", Joy, self.cb_joy, queue_size=1)
        self.sub_joy = rospy.Subscriber("/fake_joy", Joy, self.cb_joy, queue_size=1)
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.pub_joy_1 = rospy.Publisher("/wamv/joy", Joy, queue_size=1)
        self.pub_joy_2 = rospy.Publisher("/wamv2/joy", Joy, queue_size=1)
        self.pub_joy_3 = rospy.Publisher("/wamv3/joy", Joy, queue_size=1)
        self.pub_joy_4 = rospy.Publisher("/wamv4/joy", Joy, queue_size=1)

        self.pub_mode = rospy.Publisher("/control_mode", UInt8MultiArray, queue_size=10)
        self.sub_wamv_mode = rospy.Subscriber("/wamv/control_mode", UInt8, self.cb_wamv_mode, queue_size=1)
        self.sub_wamv2_mode = rospy.Subscriber("/wamv2/control_mode", UInt8, self.cb_wamv2_mode, queue_size=1)
        self.sub_wamv3_mode = rospy.Subscriber("/wamv3/control_mode", UInt8, self.cb_wamv3_mode, queue_size=1)
        self.sub_wamv4_mode = rospy.Subscriber("/wamv4/control_mode", UInt8, self.cb_wamv4_mode, queue_size=1)
        
        self.vr_joy = None
        self.vr_to_joy = Joy()
        self.vr_to_joy.header.frame_id = "/dev/input/js0"
        self.vr_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vr_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_once = True
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)   
        
        self.publisher_to_use = None
        self.index = None

        # mode management 
        # 3: DP 
        # 6: Manual
        # 7: Auto
        self.mode = UInt8MultiArray()        
        self.mode.data = [0, 0, 0]
        self.last_button_pressed = None

        self.last_button_pressed = None
            
    def cb_wamv_mode(self, msg):
        self.wamv_mode = msg.data
        
    def cb_wamv2_mode(self, msg):
        self.wamv2_mode = msg.data
        
    def cb_wamv3_mode(self, msg):
        self.wamv3_mode = msg.data
        
    def cb_wamv4_mode(self, msg):
        self.wamv4_mode = msg.data
           
    def cb_joy(self, msg):
        self.vr_joy = msg
            
    def inital_DP(self):
        # DP in th beginning
        self.vr_to_joy.buttons[3] = 1
        self.vr_to_joy.buttons[7] = 1
        
        #wamv2 DP
        self.vr_to_joy.axes[2] == 2
        self.pub_joy_2.publish(self.vr_to_joy)
        #wamv DP
        self.vr_to_joy.axes[2] = 1
        self.pub_joy_1.publish(self.vr_to_joy)
        #wamv3 DP
        self.vr_to_joy.axes[2] = 3
        self.pub_joy_3.publish(self.vr_to_joy)
        #wamv4 DP
        self.vr_to_joy.axes[2] = 4
        self.pub_joy_4.publish(self.vr_to_joy)
        
        self.mode.data = [3, 3, 3]
        self.pub_once = False

    def vr_translate_into_joy(self):
        #button 
        self.vr_to_joy.buttons[4] = self.vr_joy.buttons[0] # LB :Manual
        self.vr_to_joy.buttons[7] = self.vr_joy.buttons[1] # Start: RL
        self.vr_to_joy.buttons[3] = self.vr_joy.buttons[2] # Y: DP
        self.vr_to_joy.buttons[6] = self.vr_joy.buttons[3] # Back: Estop
        self.vr_to_joy.buttons[0] = self.vr_joy.buttons[7] # A :sync

        # self.vr_to_joy.buttons[0] = msg.buttons[5] 
        # self.vr_to_joy.buttons[1] = msg.buttons[6] # B 
        # print(self.vr_joy)

        #axes
        self.vr_to_joy.axes[1] = self.vr_joy.axes[4] # left stick forward/backward
        self.vr_to_joy.axes[3] = self.vr_joy.axes[5] # right stick right/left
        self.vr_to_joy.axes[2] = int(self.vr_joy.axes[1]) # robot
        self.vr_to_joy.axes[5] = int(self.vr_joy.axes[0]) # user_id
    
    def pub_actor(self, current_button_pressed):
        # up: wamv1, down: wamv2, left: wamv3, right: wamv4
        if self.vr_to_joy.axes[2] == 2:
            self.publisher_to_use = 2

        elif self.vr_to_joy.axes[2] == 3:
            self.publisher_to_use = 3

        elif self.vr_to_joy.axes[2] == 4:
            self.publisher_to_use = 4

        else:
            pass 
        
        return current_button_pressed
        
    def change_mode(self, current_button_pressed):
        if current_button_pressed is None:
            return
        
        if self.publisher_to_use in [1, 2]:
            self.index = 0 
        elif self.publisher_to_use == 3 :
            self.index = 1
        elif self.publisher_to_use == 4 :
            self.index = 2
                
        if self.index != None :
            self.mode.data[self.index] = current_button_pressed

    def change_mode_DP(self):
        try:
            if self.wamv_mode == 3 :
                self.mode.data[0] = 3
        except:
            pass
        
        try:
            if self.wamv2_mode == 3 :
                self.mode.data[0] = 3
        except:
            pass
        
        try:
            if self.wamv3_mode == 3 :
                self.mode.data[1] = 3
        except:
            pass
        
        try:
            if self.wamv4_mode == 3 :
                self.mode.data[2] = 3
        except:
            pass
    

    def timer_callback(self,event):
        if self.vr_joy is not None:
            self.vr_to_joy.header.stamp = rospy.Time.now()

            if self.pub_once:
                self.pub_shutdown_joy_remap_joy.publish(True)
                self.inital_DP()

            self.vr_translate_into_joy()

            # for transform wamv pose to wamv2 pose
            self.pub_joy.publish(self.vr_to_joy)

            # Detect mode change with single button press
            if self.vr_to_joy.buttons[6] == 1 :  # Estop mode
                current_button_pressed = 1
            elif self.vr_to_joy.buttons[4] == 1 :  # Manual mode
                current_button_pressed = 4
                self.vr_to_joy.buttons[6] = 1
            elif self.vr_to_joy.buttons[7] == 1 :  # Auto (RL) mode
                current_button_pressed = 7
            elif self.vr_to_joy.buttons[3] == 1 :  # DP mode
                current_button_pressed = 3
                self.vr_to_joy.buttons[7] = 1
            else: 
                current_button_pressed = None

            current_button_pressed = self.pub_actor(current_button_pressed)
            self.change_mode(current_button_pressed)
                    
            # Special case for DP mode in Nav_DP file, which will auto change to DP after RL
            self.change_mode_DP()
            
            
            # Digital twin share the same command 
            if self.publisher_to_use == 2:
                if self.mode.data[0] == 4:
                    self.vr_to_joy.buttons[4] = 1
                self.vr_to_joy.axes[2] = 2
                self.pub_joy_2.publish(self.vr_to_joy)
                self.vr_to_joy.axes[2] = 1
                self.pub_joy_1.publish(self.vr_to_joy)
                
            elif self.publisher_to_use == 3:
                if self.mode.data[1] == 4:
                     self.vr_to_joy.buttons[4] = 1
                self.pub_joy_3.publish(self.vr_to_joy)
            elif self.publisher_to_use == 4:
                if self.mode.data[2] == 4:
                    self.vr_to_joy.buttons[4] = 1
                self.pub_joy_4.publish(self.vr_to_joy)
            else: 
                pass
            
            self.pub_mode.publish(self.mode)
            print(f'Pub {self.mode.data}')
            
            if current_button_pressed is not None:
                self.last_button_pressed = current_button_pressed

if __name__ == '__main__':
    rospy.init_node('vr_remap_joy')
    vr_remap_joy = VR_remap_joy()
    # rospy.spin()
    while not rospy.is_shutdown():
        if vr_remap_joy.sub_joy.get_num_connections() == 0:
            print("No messages received on /vr_teleop yet...")
    rospy.sleep(0.1) 