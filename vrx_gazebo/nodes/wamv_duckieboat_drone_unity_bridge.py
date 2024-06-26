#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Empty

class DroneUnityBridge:
    def __init__(self):

        self.robot_start_number = 6
        self.robot_amount = 1
        self.robot_select = 6

        self.linear_scaling_factor = rospy.get_param("~linear_scaling_factor", 1.0)
        self.angular_scaling_factor = rospy.get_param("~angular_scaling_factor", 1.0)
        self.cmd_vel = Twist()
        # Create an Empty message
        self.empty_msg = Empty()
        self.takeoff = False

        # VR to ROS side
        self.sub_vr = rospy.Subscriber("vr/joystick", Joy, self.vr_callback, queue_size=10)
        self.pub_cmd_vel_list = [
            rospy.Publisher(f"cmd_vel", Twist, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        # Create a publisher for the /drone/takeoff topic
        self.pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        
        # ROS to VR  side
        self.sub_model = rospy.Subscriber("gazebo/model_states/throttle", ModelStates, self.model_callback, queue_size=10)
        self.robot_pose_list = [PoseStamped() for i in range(self.robot_amount)]
        self.pub_robot_pose_list = [
            rospy.Publisher(f"vr/drone/pose", PoseStamped, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        self.pub_camera_pose = rospy.Publisher(f"vr/camera/pose", PoseStamped, queue_size=10)

        # [s, rx, ry, mode]
    
    def vr_callback(self, msg):
        self.robot_select = int(msg.axes[0])
        if self.robot_select == 6:
            self.cmd_vel.angular.z = -msg.axes[1]
            self.cmd_vel.linear.x = msg.axes[2]
            self.cmd_vel.linear.z = msg.axes[4] #updown
            if self.takeoff is False and msg.axes[4] > 0.3 :
                self.pub_takeoff.publish(self.empty_msg)
                self.takeoff = True
            self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)

    def model_callback(self, msg: ModelStates):
        robot_index = [msg.name.index(f"sjtu_drone") for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)]
        for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount):
            self.robot_pose_list[i - self.robot_start_number].header.stamp = rospy.Time.now()
            self.robot_pose_list[i - self.robot_start_number].header.frame_id = "map"
            self.robot_pose_list[i - self.robot_start_number].pose = msg.pose[robot_index[i - self.robot_start_number]]

            self.pub_robot_pose_list[i - self.robot_start_number].publish(self.robot_pose_list[i - self.robot_start_number])

            if i == self.robot_select:
                self.pub_camera_pose.publish(self.robot_pose_list[i - self.robot_start_number])

class DuckieboatUnityBridge:
    def __init__(self):

        self.robot_start_number = 5
        self.robot_amount = 1
        self.robot_select = 5

        self.robot_ns = "duckie_alpha"

        self.linear_scaling_factor = rospy.get_param("~linear_scaling_factor", 1.0)
        self.angular_scaling_factor = rospy.get_param("~angular_scaling_factor", 1.0)

        self.cmd_vel = Twist()
        self.thruster_cmd = [Float32() for i in range(2)]

        # VR to ROS side
        self.sub_vr = rospy.Subscriber("vr/joystick", Joy, self.vr_callback, queue_size=10)
        self.pub_cmd_vel_list = [
            rospy.Publisher(f"{self.robot_ns}/cmd_vel", Twist, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        self.pub_thruster_cmd_list = [
            [
                rospy.Publisher(f"{self.robot_ns}/thrusters/left_rear_thrust_cmd", Float32, queue_size=10),
                rospy.Publisher(f"{self.robot_ns}/thrusters/right_rear_thrust_cmd", Float32, queue_size=10),
            ]
            for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        
        # ROS to VR  side
        self.sub_model = rospy.Subscriber("gazebo/model_states/throttle", ModelStates, self.model_callback, queue_size=10)
        self.robot_pose_list = [PoseStamped() for i in range(self.robot_amount)]
        self.pub_robot_pose_list = [
            rospy.Publisher(f"vr/{self.robot_ns}/pose", PoseStamped, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        self.pub_camera_pose = rospy.Publisher(f"vr/camera/pose", PoseStamped, queue_size=10)

        # [s, rx, ry, mode]
    
    def vr_callback(self, msg):
        self.robot_select = int(msg.axes[0])
        if self.robot_select == 5:
            self.cmd_vel.linear.x = msg.axes[2]
            self.cmd_vel.angular.z = msg.axes[1]
            self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)
            
            self.thruster_cmd[0].data = self.cmd_vel.linear.x + self.cmd_vel.angular.z
            self.thruster_cmd[1].data = self.cmd_vel.linear.x - self.cmd_vel.angular.z
            
            for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount):
                for j in range(2):
                    if i == self.robot_select:
                        self.pub_thruster_cmd_list[i - self.robot_start_number][j].publish(self.thruster_cmd[j])
                    else:
                        self.pub_thruster_cmd_list[i - self.robot_start_number][j].publish(Float32())

    def model_callback(self, msg: ModelStates):
        robot_index = [msg.name.index(f"{self.robot_ns}") for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)]
        for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount):
            self.robot_pose_list[i - self.robot_start_number].header.stamp = rospy.Time.now()
            self.robot_pose_list[i - self.robot_start_number].header.frame_id = "map"
            self.robot_pose_list[i - self.robot_start_number].pose = msg.pose[robot_index[i - self.robot_start_number]]

            self.pub_robot_pose_list[i - self.robot_start_number].publish(self.robot_pose_list[i - self.robot_start_number])

            if i == self.robot_select:
                self.pub_camera_pose.publish(self.robot_pose_list[i - self.robot_start_number])

class WAMVUnityBridge:
    def __init__(self):

        self.robot_start_number = rospy.get_param("~robot_start_number", 1)
        self.robot_amount = rospy.get_param("~robot_amount", 4)
        self.robot_select = self.robot_start_number

        self.robot_ns = rospy.get_param("~namespace", "wamv")

        self.linear_scaling_factor = rospy.get_param("~linear_scaling_factor", 1.0)
        self.angular_scaling_factor = rospy.get_param("~angular_scaling_factor", 1.0)

        self.cmd_vel = Twist()
        self.thruster_cmd = [Float32() for i in range(4)]

        # VR to ROS side
        self.sub_vr = rospy.Subscriber("vr/joystick", Joy, self.vr_callback, queue_size=10)
        self.pub_cmd_vel_list = [
            rospy.Publisher(f"{self.robot_ns}{i}/cmd_vel", Twist, queue_size=10) for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
        ]
        self.pub_thruster_cmd_list = [
            [
                rospy.Publisher(f"{self.robot_ns}{i}/thrusters/left_thrust_cmd", Float32, queue_size=10),
                rospy.Publisher(f"{self.robot_ns}{i}/thrusters/right_thrust_cmd", Float32, queue_size=10),
                rospy.Publisher(f"{self.robot_ns}{i}/thrusters/left_lateral_thrust_cmd", Float32, queue_size=10),
                rospy.Publisher(f"{self.robot_ns}{i}/thrusters/right_lateral_thrust_cmd", Float32, queue_size=10),
            ]
            for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount)
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
        if self.robot_select < 5:
            self.cmd_vel.linear.x = msg.axes[2]
            self.cmd_vel.angular.z = -msg.axes[1]
            self.pub_cmd_vel_list[self.robot_select - self.robot_start_number].publish(self.cmd_vel)
            
            self.thruster_cmd[0].data = self.cmd_vel.linear.x
            self.thruster_cmd[1].data = self.cmd_vel.linear.x
            self.thruster_cmd[2].data = self.cmd_vel.angular.z
            self.thruster_cmd[3].data = -self.cmd_vel.angular.z

            for i in range(self.robot_start_number, self.robot_start_number + self.robot_amount):
                for j in range(4):
                    if i == self.robot_select:
                        self.pub_thruster_cmd_list[i - self.robot_start_number][j].publish(self.thruster_cmd[j])
                    else:
                        self.pub_thruster_cmd_list[i - self.robot_start_number][j].publish(Float32())

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
    rospy.init_node("wamv_duckieboat_unity_bridge")
    wamv_unity_bridge = WAMVUnityBridge()
    duckieboat_unity_bridge = DuckieboatUnityBridge()
    drone_unity_bridge = DroneUnityBridge()
    rospy.spin()
