#!/usr/bin/env python3
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Node():
    def __init__(self):
        self.cmd_vel = Twist()
        
        # Publisher
        self.cmd_pub = rospy.Publisher("/boat1/cmd_vel",Twist,queue_size=10)


        # Subscriber
        self.sub_cmd = rospy.Subscriber("/boat1/pub2ros/thrusters/rudder", Float64, self.cb_rudder, queue_size=1)
        self.sub_joy = rospy.Subscriber("/boat1/pub2ros/thrusters/speed", Float64,self.cbspeed, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)

    def cb_publish(self, event):
        print(self.cmd_vel)
        self.cmd_pub.publish(self.cmd_vel)

    def cb_rudder(self, data):
        #print(data)
        self.cmd_vel.angular.z = data.data/100
    
    def cbspeed(self,data):
        self.cmd_vel.linear.x = data.data/100


if __name__ == '__main__':

    rospy.init_node('moostocmdvel', anonymous=True)

    node=Node()


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
