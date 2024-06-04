#!/usr/bin/env python3
import fix_python3_path
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8MultiArray

control_mode = UInt8MultiArray()
control_mode.data = [3,3,3]

# manual, auto, dp, estop
vr_mode_contorl_mode = [4,7,3,1]
# [0,0,0,1]

control_mode_pub = None

def joy_cb(msg):
    try:
        if 2 <= msg.axes[1] <= 4:
            # control_mode.data[int(msg.axes[1])-2] = vr_mode_contorl_mode[int(msg.axes[2])]
            control_mode.data[int(msg.axes[1])-2] = int(msg.axes[2])

        # control_mode.data[int(msg.axes[1])-2] = vr_mode_contorl_mode[msg.buttons[0:4].index(1)]
    except ValueError:
        pass
    if control_mode_pub:
        control_mode_pub.publish(control_mode)
    

if __name__ == "__main__":
    rospy.init_node("vr_teleop_to_control_mode")
    rospy.Subscriber("/vr_teleop", Joy, joy_cb, queue_size=10) 
    control_mode_pub = rospy.Publisher("/control_mode", UInt8MultiArray, queue_size=10)
    rospy.spin()
