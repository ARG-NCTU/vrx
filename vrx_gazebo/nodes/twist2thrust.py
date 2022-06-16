#!/usr/bin/env python3
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class Node():
    def __init__(self,linear_scaling,angular_scaling,keyboard=False):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.left_pub = None
        self.right_pub = None
        self.left_msg =Float32()
        self.right_msg =Float32()
        self.left_lateral_msg =Float32()
        self.right_lateral_msg =Float32()
        self.keyboard = keyboard
        self.auto = 0
        
        # Publisher
        self.left_pub = rospy.Publisher("left_cmd",Float32,queue_size=10)
        self.right_pub = rospy.Publisher("right_cmd",Float32,queue_size=10)
        self.left_lateral_pub = rospy.Publisher("left_lateral_cmd",Float32,queue_size=10)
        self.right_lateral_pub = rospy.Publisher("right_lateral_cmd",Float32,queue_size=10)

        # Subscriber
        self.sub_cmd = rospy.Subscriber("/boat1/cmd_vel", Twist, self.cb_cmd, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cbJoy, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)

    def cb_publish(self, event):
        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)
        self.left_lateral_pub.publish(self.left_lateral_msg)
        self.right_lateral_pub.publish(self.right_lateral_msg)
    def cb_cmd(self, data):
        if self.auto:
            self.left_msg.data = data.linear.x
            self.right_msg.data = data.linear.x
            self.left_lateral_msg = -1*data.angular.z
            self.right_lateral_msg = data.angular.z
            print(self.left_lateral_msg)
            print(self.right_lateral_msg)
    
    def cbJoy(self,data):
        if(data.buttons[7]==1) and not self.auto:
            self.auto = 1
            rospy.loginfo("going auto")
        elif(data.buttons[6]==1) and self.auto:
            self.auto = 0
            rospy.loginfo("going manual")

        if not self.auto:
            self.left_msg.data = data.axes[1]*self.linear_scaling
            self.right_msg.data = data.axes[1]*self.linear_scaling 
            self.left_lateral_msg = data.axes[3]*self.angular_scaling
            self.right_lateral_msg = -1*data.axes[3]*self.angular_scaling

if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',1)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',1)

    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))


    key = '--keyboard' in sys.argv
    node=Node(linear_scaling,angular_scaling,keyboard=key)


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
