#!/usr/bin/env python3
import fix_python3_path
from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Bool
class Joy_remap_joy:
    # This script remaps the joy to the joy topic
    # For mixed teleop
    def __init__(self):
        
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)

        self.pub_joy_1 = rospy.Publisher("/wamv/joy", Joy, queue_size=1)
        self.pub_joy_2 = rospy.Publisher("/wamv2/joy", Joy, queue_size=1)
        self.pub_joy_3 = rospy.Publisher("/wamv3/joy", Joy, queue_size=1)
        self.pub_joy_4 = rospy.Publisher("/wamv4/joy", Joy, queue_size=1)

        self.joy_to_joy = Joy()
        self.joy_to_joy.header.frame_id = "/dev/input/js0"
        self.joy_to_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self.sub_shutdown_joy_remap_joy = rospy.Subscriber("/shutdown_joy_remap_joy", Bool, self.shutdown_cb, queue_size=1)
        self.flag = False
        self.first_time = True
        self.publisher_to_use = None
        self.index = None

        # mode management 
        # 3: DP 
        # 6: Manual
        # 7: Auto
        self.mode = [0, 0, 0]
        self.last_button_pressed = None

    def shutdown_cb(self, msg):
        self.flag = msg.data
        
    def cb_joy(self, msg):
        self.joy_to_joy.header.stamp = rospy.Time.now()        
        if self.first_time:
            self.inital_DP()
        
        self.joy_to_joy.axes = list(msg.axes)
        self.joy_to_joy.buttons = list(msg.buttons)

        # # Check for mode switch button presses
        # if self.joy_to_joy.buttons[6] == 1 and self.last_button_pressed != 6:  # Manual mode
        #     current_button_pressed = 6
        # elif self.joy_to_joy.buttons[7] == 1 and self.last_button_pressed != 7:  # Auto mode
        #     current_button_pressed = 7
        # elif self.joy_to_joy.buttons[3] == 1 and self.last_button_pressed != 3:  # DP mode
        #     current_button_pressed = 3
        #     self.joy_to_joy.buttons[7] = 1
        # else:
        #     current_button_pressed = None
        
        # Check for mode switch button presses
        if self.joy_to_joy.buttons[6] == 1 :  # Manual mode
            current_button_pressed = 6
        elif self.joy_to_joy.buttons[7] == 1 :  # Auto mode
            current_button_pressed = 7
        elif self.joy_to_joy.buttons[3] == 1 :  # DP mode
            current_button_pressed = 3
            self.joy_to_joy.buttons[7] = 1
        else:
            current_button_pressed = None

        self.pub_actor(current_button_pressed)

        # For real wamv manual control
        if self.mode[0] == 6 and self.publisher_to_use == 2:
            self.joy_to_joy.buttons[4] = 1
            
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
        
        if current_button_pressed is not None:
            self.last_button_pressed = current_button_pressed
    
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
        self.mode = [3, 3, 3]
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

        self.change_mode(current_button_pressed)

    
    def change_mode(self, current_button_pressed):
        if current_button_pressed is None:
            return
        if self.publisher_to_use == 2:
            self.index = 0
        elif self.publisher_to_use == 3 :
            self.index = 1
        elif self.publisher_to_use == 4 :
            self.index = 2
        else :
            pass 
        if self.index != None :
            self.mode[self.index] = current_button_pressed
        print(self.mode)
        print(f'Pub {self.publisher_to_use} in mode {self.mode[self.index]}')

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