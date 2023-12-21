#!/usr/bin/env python
from sensor_msgs.msg import Joy
import rospy
 
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
        self.joy_to_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.publisher_to_use = None


    def cb_joy(self, msg):
        self.joy_to_joy.header.stamp = rospy.Time.now()
        self.joy_to_joy.axes = list(msg.axes)
        self.joy_to_joy.buttons = list(msg.buttons)

        #DP  then switch to Auto
        if self.joy_to_joy.buttons[3] == 1:
            self.joy_to_joy.buttons[7] = 1
        
        # up: wamv1, down: wamv2, left: wamv3, right: wamv4
        if self.joy_to_joy.axes[7] == 1:
            self.joy_to_joy.axes[2] = 1
            self.publisher_to_use = 1
            print('wamv1')

        elif self.joy_to_joy.axes[7] == -1:
            self.joy_to_joy.axes[2] = 2
            self.publisher_to_use = 2
            print('wamv2')

        elif self.joy_to_joy.axes[6] == 1:
            self.joy_to_joy.axes[2] = 3
            self.publisher_to_use = 3
            print('wamv3')
        elif self.joy_to_joy.axes[6] == -1:
            self.joy_to_joy.axes[2] = 4
            self.publisher_to_use = 4
            print('wamv4')
            
        print('pub:',self.publisher_to_use)
        # # Keep publishing on the selected topic until a condition changes
        if self.publisher_to_use == 1:
            self.joy_to_joy.axes[2] = 1
            self.pub_joy_1.publish(self.joy_to_joy)
            self.joy_to_joy.axes[2] = 2
            self.pub_joy_2.publish(self.joy_to_joy)

        elif self.publisher_to_use == 2:
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

if __name__ == '__main__':
    rospy.init_node('joy_remap_joy')
    joy_remap_joy = Joy_remap_joy()
    rospy.spin()
    
    rospy.sleep(0.1) 