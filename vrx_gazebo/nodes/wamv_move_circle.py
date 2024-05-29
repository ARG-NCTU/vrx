#! /usr/bin/env python

import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Float64

FAIL = 0
RUNNING = 1
SUCCESS = 2


class WamvController:
    def __init__(self):
        rospy.init_node("wamv_move_circle")
        self.publish_rate = rospy.get_param("~publish_rate", 10)

        self.kp_linear = rospy.get_param("~kp_linear", 4.0)
        self.ki_linear = rospy.get_param("~ki_linear", 1.0)
        self.kd_linear = rospy.get_param("~kd_linear", 0.8)

        self.kp_angular = rospy.get_param("~kp_angular", 20.0)
        self.ki_angular = rospy.get_param("~ki_angular", 6.0)
        self.kd_angular = rospy.get_param("~kd_angular", 0.1)

        self.target_linear_speed = rospy.get_param("~target_linear_speed", 0.5)
        self.target_angular_speed = rospy.get_param("~target_angular_speed", 0.025)

        self.wamv_move_circle_status_pub = rospy.Publisher("/wamv_move_circle_status", Status, queue_size=10)
        self.left_pub = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size=10)
        self.right_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size=10)
        self.left_lateral_pub = rospy.Publisher("/wamv/thrusters/left_lateral_thrust_cmd", Float32, queue_size=10)
        self.right_lateral_pub = rospy.Publisher("/wamv/thrusters/right_lateral_thrust_cmd", Float32, queue_size=10)

        rospy.Subscriber("/wamv_move_circle_active", Active, self.active_callback)
        rospy.Subscriber("/gazebo/wamv/twist", TwistStamped, self.twist_callback)
        rospy.Subscriber("/wamv/target_linear_speed", Float64, self.target_linear_speed_callback)
        rospy.Subscriber("/wamv/target_angular_speed", Float64, self.target_angular_speed_callback)

        self.prev_error_linear = 0
        self.integral_linear = 0
        self.prev_error_angular = 0
        self.integral_angular = 0

        self.current_linear_speed = 0
        self.current_angular_speed = 0

        self.active = False

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

    def twist_callback(self, msg):
        self.current_linear_speed = msg.twist.linear.x
        self.current_angular_speed = msg.twist.angular.z

    def target_linear_speed_callback(self, msg):
        self.target_linear_speed = msg.data

    def target_angular_speed_callback(self, msg):
        self.target_angular_speed = msg.data

    def pid_control(self, target, current, prev_error, integral, kp, ki, kd):
        error = target - current
        integral += error * (1.0 / self.publish_rate)
        derivative = (error - prev_error) / (1.0 / self.publish_rate)

        output = kp * error + ki * integral + kd * derivative
        prev_error = error

        return output, prev_error, integral

    def timer_callback(self, event):
        if not self.active:
            linear_thrust = 0
            angular_thrust = 0
            # Set I to zero when not active
            self.integral_linear = 0
            self.integral_angular = 0
            return

        linear_thrust, self.prev_error_linear, self.integral_linear = self.pid_control(
            self.target_linear_speed,
            self.current_linear_speed,
            self.prev_error_linear,
            self.integral_linear,
            self.kp_linear,
            self.ki_linear,
            self.kd_linear,
        )

        angular_thrust, self.prev_error_angular, self.integral_angular = self.pid_control(
            self.target_angular_speed,
            self.current_angular_speed,
            self.prev_error_angular,
            self.integral_angular,
            self.kp_angular,
            self.ki_angular,
            self.kd_angular,
        )

        self.left_pub.publish(linear_thrust)
        self.right_pub.publish(linear_thrust)
        self.left_lateral_pub.publish(angular_thrust)
        self.right_lateral_pub.publish(-angular_thrust)

        print("Linear Thrust: {}, Angular Thrust: {}          ".format(linear_thrust, angular_thrust))

    def active_callback(self, msg):
        self.active = msg.active
        status = Status()
        status.id = msg.id
        status.status = SUCCESS
        self.wamv_move_circle_status_pub.publish(status)


if __name__ == "__main__":
    WamvController()
    rospy.spin()
