#!/usr/bin/env python3
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class Node():
    def __init__(self):
        self.cmd = Twist()
        self.count = 0
        self.cmd.linear.x = 1
        
        # Publisher
        self.cmd_pub = rospy.Publisher("/X1/cmd_vel",Twist,queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)


    def cb_publish(self, event):
        self.cmd_pub.publish(self.cmd)
        if(self.count > 100):
            self.cmd.linear.x = self.cmd.linear.x - 0.1
            self.count = 0

        self.count = self.count + 1
        #print(self.count)

if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.6)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.65)

    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))


    key = '--keyboard' in sys.argv
    node=Node()


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
