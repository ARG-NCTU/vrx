#!/usr/bin/env python

import numpy as np
import rospy
from mavros_msgs.msg import RCOut
from sensor_msgs.msg import JointState


class JointStatePublisher:
    def __init__(self):
        rospy.init_node("if750a_joint_state_publisher", anonymous=True)
        self.joint_state = JointState()
        self.rc_out_sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_out_callback)
        self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.rate = rospy.get_param("~rate", 30.0)
        self.timer = rospy.Timer(rospy.Duration(1 / self.rate), self.timer_callback)

        self.motor_position = np.zeros(5)
        self.motor_speed = np.zeros(5)

    def rc_out_callback(self, msg):
        self.motor_speed[1:5] = np.array(msg.channels[0:4]) - 900.0
        self.motor_speed[3:5] = -self.motor_speed[3:5]
        print(self.motor_speed)
        print(self.motor_position)

    def timer_callback(self, event):
        self.motor_position[1:5] += 2.0 * self.motor_speed[1:5] / self.rate
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = ["imu_joint", "rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"]
        self.joint_state.position = list(self.motor_position)
        self.joint_state.velocity = list(self.motor_speed)
        self.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states_pub.publish(self.joint_state)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    joint_state_publisher = JointStatePublisher()
    joint_state_publisher.run()
