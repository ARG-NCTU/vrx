#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def main():
    rospy.init_node("wamv_go_circle")
    
    left_pub = rospy.Publisher("/wamv/thrusters/left_thrust_cmd",Float32,queue_size=10)
    right_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd",Float32,queue_size=10)
    left_lateral_pub = rospy.Publisher("/wamv/thrusters/left_lateral_thrust_cmd",Float32,queue_size=10)
    right_lateral_pub = rospy.Publisher("/wamv/thrusters/right_lateral_thrust_cmd",Float32,queue_size=10)
    
    def timer_callback(event):
        left_pub.publish(0.6)
        right_pub.publish(0.6)
        left_lateral_pub.publish(0.3)
        right_lateral_pub.publish(-0.3)
        
    rospy.Timer(rospy.Duration(0.1), timer_callback)

if __name__ == "__main__":
    main()
    rospy.spin()
