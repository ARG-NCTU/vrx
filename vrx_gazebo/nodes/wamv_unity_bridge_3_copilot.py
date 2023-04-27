#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates


class WAMVUnityBridge_3_copilot:
    def __init__(self):

        self.robot_start_number = rospy.get_param("~robot_start_number", 1)
        self.robot_amount = rospy.get_param("~robot_amount", 5)
        self.robot_select = self.robot_start_number

        self.robot_ns = rospy.get_param("~namespace", "wamv")

        self.linear_scaling_factor = rospy.get_param("~linear_scaling_factor", 1.0)
        self.angular_scaling_factor = rospy.get_param("~angular_scaling_factor", 1.0)

        self.cmd_vel = Twist()
        self.thruster_cmd = [Float32() for i in range(4)]

        # VR to ROS side
        self.sub_vr = rospy.Subscriber("vr/joystick", Joy, self.vr_callback, queue_size=10)
        self.sub_vr = rospy.Subscriber("vr/joystick2", Joy, self.vr_callback_2, queue_size=10)
        self.sub_vr = rospy.Subscriber("vr/joystick3", Joy, self.vr_callback_3, queue_size=10)
        self.pub_cmd_vel_list = [
            rospy.Publisher(f"{self.robot_ns}{i}/cmd_vel_human", Twist, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]

        # ROS to VR  side
        self.sub_model = rospy.Subscriber("gazebo/model_states/throttle", ModelStates, self.model_callback, queue_size=10)
        self.robot_pose_list = [PoseStamped() for i in range(self.robot_amount)]
        self.pub_robot_pose_list = [
            rospy.Publisher(f"vr/{self.robot_ns}{i}/pose", PoseStamped, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        self.pub_camera_pose = rospy.Publisher(f"vr/camera/pose", PoseStamped, queue_size=10)

        # [s, rx, ry, mode]

    def vr_callback(self, msg):
        self.robot_select = int(msg.axes[0])
        self.cmd_vel.linear.x = msg.axes[2]
        self.cmd_vel.angular.z = -msg.axes[1]
        self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)

    def vr_callback_2(self, msg):
        self.robot_select = int(msg.axes[0])
        self.cmd_vel.linear.x = msg.axes[2]
        self.cmd_vel.angular.z = -msg.axes[1]
        self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)

    def vr_callback_3(self, msg):
        self.robot_select = int(msg.axes[0])
        self.cmd_vel.linear.x = msg.axes[2]
        self.cmd_vel.angular.z = -msg.axes[1]
        self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)

    def model_callback(self, msg: ModelStates):
        robot_index = [msg.name.index(f"{self.robot_ns}{i}") for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)]
        for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount):
            self.robot_pose_list[i - self.robot_start_number].header.stamp = rospy.Time.now()
            self.robot_pose_list[i - self.robot_start_number].header.frame_id = "map"
            self.robot_pose_list[i - self.robot_start_number].pose = msg.pose[robot_index[i - self.robot_start_number]]

            self.pub_robot_pose_list[i - self.robot_start_number].publish(self.robot_pose_list[i - self.robot_start_number])

            if i == self.robot_select:
                self.pub_camera_pose.publish(self.robot_pose_list[i - self.robot_start_number])


if __name__ == "__main__":
    rospy.init_node("wamv_unity_bridge_3_copilot")
    wamv_unity_bridge = WAMVUnityBridge_3_copilot()
    rospy.spin()
