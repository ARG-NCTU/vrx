#!/usr/bin/env python3
import fix_python3_path
from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Bool,UInt8MultiArray , UInt8
class Joy_remap_joy:
    # This script remaps the joy to the joy topic
    # For mixed teleop
    def __init__(self):
        
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)

        self.pub_joy_1 = rospy.Publisher("/wamv/joy", Joy, queue_size=1)
        self.pub_joy_2 = rospy.Publisher("/wamv2/joy", Joy, queue_size=1)
        self.pub_joy_3 = rospy.Publisher("/wamv3/joy", Joy, queue_size=1)
        self.pub_joy_4 = rospy.Publisher("/wamv4/joy", Joy, queue_size=1)
        self.pub_mode = rospy.Publisher("/control_mode", UInt8MultiArray, queue_size=10)
        self.pub_mode_to_ar = rospy.Publisher("/mode_to_ar", UInt8MultiArray, queue_size=10)
        
        self.sub_wamv_mode = rospy.Subscriber("/wamv/control_mode", UInt8, self.cb_wamv_mode, queue_size=1)
        self.sub_wamv2_mode = rospy.Subscriber("/wamv2/control_mode", UInt8, self.cb_wamv2_mode, queue_size=1)
        self.sub_wamv3_mode = rospy.Subscriber("/wamv3/control_mode", UInt8, self.cb_wamv3_mode, queue_size=1)
        self.sub_wamv4_mode = rospy.Subscriber("/wamv4/control_mode", UInt8, self.cb_wamv4_mode, queue_size=1)
        
        self.sub_wamv_auto = rospy.Subscriber("/auto_state", Bool, self.cb_wamv_auto, queue_size=1)
        self.sub_wamv2_auto = rospy.Subscriber("/wamv2/auto_state", Bool, self.cb_wamv2_auto, queue_size=1)
        self.sub_wamv3_auto = rospy.Subscriber("/wamv3/auto_state", Bool, self.cb_wamv3_auto, queue_size=1)
        self.sub_wamv4_auto = rospy.Subscriber("/wamv4/auto_state", Bool, self.cb_wamv4_auto, queue_size=1)
        
        self.sub_wamv_estop = rospy.Subscriber("/stop_state", Bool, self.cb_wamv_estop, queue_size=1)
        self.sub_wamv2_estop = rospy.Subscriber("/wamv2/stop_state", Bool, self.cb_wamv2_estop, queue_size=1)
        self.sub_wamv3_estop = rospy.Subscriber("/wamv3/stop_state", Bool, self.cb_wamv3_estop, queue_size=1)
        self.sub_wamv4_estop = rospy.Subscriber("/wamv4/stop_state", Bool, self.cb_wamv4_estop, queue_size=1)
        
        self.joy = None
        self.joy_to_joy = Joy()
        self.joy_to_joy.header.frame_id = "/dev/input/js0"
        self.joy_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self.sub_shutdown_joy_remap_joy = rospy.Subscriber("/shutdown_joy_remap_joy", Bool, self.shutdown_cb, queue_size=1)
        self.flag = False
        self.first_time = True
        self.publisher_to_use = None
        self.index = None

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)   

        # mode management 
        # 0 :Manual
        # 1: Auto
        # 2: DP
        # 3: Estop
        self.mode = UInt8MultiArray()        
        self.mode.data = [0, 0, 0]
        self.last_button_pressed = None

    def shutdown_cb(self, msg):
        self.flag = msg.data
        
    def cb_wamv_mode(self, msg):
        self.wamv_mode = msg.data
        
    def cb_wamv2_mode(self, msg):
        self.wamv2_mode = msg.data
        
    def cb_wamv3_mode(self, msg):
        self.wamv3_mode = msg.data
        
    def cb_wamv4_mode(self, msg):
        self.wamv4_mode = msg.data
      
    def cb_joy(self, msg):
        self.joy = msg
        
    def cb_wamv_auto(self, msg):
        self.wamv_auto = msg.data
        
    def cb_wamv2_auto(self, msg):
        self.wamv2_auto = msg.data
    
    def cb_wamv3_auto(self, msg):
        self.wamv3_auto = msg.data
    
    def cb_wamv4_auto(self, msg):
        self.wamv4_auto = msg.data
    
    def cb_wamv_estop(self, msg):
        self.wamv_estop = msg.data
        
    def cb_wamv2_estop(self, msg):
        self.wamv2_estop = msg.data
        
    def cb_wamv3_estop(self, msg):
        self.wamv3_estop = msg.data
        
    def cb_wamv4_estop(self, msg):
        self.wamv4_estop = msg.data
        
    def inital_DP(self):
        # DP in th beginning
        self.joy_to_joy.buttons[3] = 1
        self.joy_to_joy.buttons[7] = 1
        
        self.joy_to_joy.axes[2] = 1
        self.pub_joy_1.publish(self.joy_to_joy)
        
        self.joy_to_joy.axes[2] = 2   
        self.pub_joy_2.publish(self.joy_to_joy)
        
        self.joy_to_joy.axes[2] = 3
        self.pub_joy_3.publish(self.joy_to_joy)
        
        self.joy_to_joy.axes[2] = 4
        self.pub_joy_4.publish(self.joy_to_joy)
        self.mode.data = [2, 2, 2]
        self.first_time = False
        
    def pub_actor(self, current_button_pressed):
        # up: wamv1, down: wamv2, left: wamv3, right: wamv4
        if self.joy_to_joy.axes[7] == -1 or self.joy_to_joy.axes[7] == 1:
            self.joy_to_joy.axes[2] = 2
            self.publisher_to_use = 2

        elif self.joy_to_joy.axes[6] == 1:
            self.joy_to_joy.axes[2] = 3
            self.publisher_to_use = 3

        elif self.joy_to_joy.axes[6] == -1:
            self.joy_to_joy.axes[2] = 4
            self.publisher_to_use = 4

        else:
            pass 

        return current_button_pressed
        
    def change_mode(self, current_button_pressed):
        
        # Determine the mode based on auto and stop states
        try:
            # Check for each WAM-V's auto and estop states and adjust mode accordingly
            if self.publisher_to_use == 2:  # wamv and wamv2
                self.index = 0
                if not self.wamv2_auto and self.wamv2_estop:
                    current_button_pressed = 3  # estop mode
                elif not self.wamv2_auto:
                    current_button_pressed = 0  # manual mode
            elif self.publisher_to_use == 3:  # wamv3
                self.index = 1
                if not self.wamv3_auto and self.wamv3_estop:
                    current_button_pressed = 3   # estop mode
                elif not self.wamv3_auto:
                    current_button_pressed = 0  # manual mode
            elif self.publisher_to_use == 4:  # wamv4
                self.index = 2
                if not self.wamv4_auto and self.wamv4_estop:
                    current_button_pressed = 3  # estop mode
                elif not self.wamv4_auto:
                    current_button_pressed = 0  # manual mode
        except AttributeError:
            pass
        
        # Update the mode for the current WAM-V
        if self.index is not None and current_button_pressed is not None:
            self.mode.data[self.index] = current_button_pressed
 
    def change_mode_DP(self):
        
        try:
            if self.wamv_mode == 3 :
                self.mode.data[0] = 2
        except:
            pass
        
        try:
            if self.wamv2_mode == 3 :
                self.mode.data[0] = 2
        except:
            pass
        
        try:
            if self.wamv3_mode == 3 :
                self.mode.data[1] = 2
        except:
            pass
        
        try:
            if self.wamv4_mode == 3 :
                self.mode.data[2] = 2
        except:
            pass
    
    def timer_callback(self, event):
        if self.joy is not None:
            self.joy_to_joy.header.stamp = rospy.Time.now()        
            if self.first_time:
                self.inital_DP()
            
            self.joy_to_joy.axes = list(self.joy.axes)
            self.joy_to_joy.buttons = list(self.joy.buttons)

            # Check for mode switch button presses
            if self.joy_to_joy.buttons[6] == 1 :  # Manual mode or estop mode 
                current_button_pressed = 3
            elif self.joy_to_joy.buttons[7] == 1 :  # Auto mode
                current_button_pressed = 1
            elif self.joy_to_joy.buttons[3] == 1 :  # DP mode
                current_button_pressed = 2
                self.joy_to_joy.buttons[7] = 1
            else:
                current_button_pressed = None

            current_button_pressed = self.pub_actor(current_button_pressed)
            self.change_mode(current_button_pressed)

            # Special case for DP mode in Nav_DP file, which will auto change to DP after RL
            self.change_mode_DP()


            # # Keep publishing on the selected topic until a condition changes
            if self.publisher_to_use == 2:
                self.joy_to_joy.axes[2] = 2
                self.pub_joy_2.publish(self.joy_to_joy)
                self.joy_to_joy.axes[2] = 1
                self.pub_joy_1.publish(self.joy_to_joy)
                
            elif self.publisher_to_use == 3:
                self.joy_to_joy.axes[2] = 3
                self.pub_joy_3.publish(self.joy_to_joy)
                
            elif self.publisher_to_use == 4:
                self.joy_to_joy.axes[2] = 4
                self.pub_joy_4.publish(self.joy_to_joy)
                
            else:       
                pass
            self.pub_mode.publish(self.mode)
            self.pub_mode_to_ar.publish(self.mode)

            print(f'Pub {self.mode.data}')

            if current_button_pressed is not None:
                self.last_button_pressed = current_button_pressed
                
    def run(self):
        while not rospy.is_shutdown():
            if self.flag:
                rospy.loginfo('Process finished')
                break
            rospy.sleep(0.1)
            
if __name__ == '__main__':
    rospy.init_node('joy_remap_joy')
    joy_remap_joy = Joy_remap_joy()
    joy_remap_joy.run()
    # rospy.spin()
    
    # rospy.sleep(0.1) 