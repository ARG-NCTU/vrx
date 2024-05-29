#!/usr/bin/env python3

import fix_python3_path
import rospy
import tf
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TwistStamped


class ModelStateToTwist:
    def __init__(self):
        rospy.init_node("relative_velocity_listener", anonymous=True)
        self.model_name = rospy.get_param("~model_name", "wamv")
        self.publish_rate = rospy.get_param("~publish_rate", 20)

        self.velocity = TwistStamped()
        self.listener = tf.TransformListener()

        self.velocity_pub = rospy.Publisher(f"/gazebo/{self.model_name}/twist", TwistStamped, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

    def model_callback(self, data):
        try:
            model_index = data.name.index(self.model_name)

            orientation = data.pose[model_index].orientation
            world_velocity = data.twist[model_index]

            linear_velocity_world = [world_velocity.linear.x, world_velocity.linear.y, world_velocity.linear.z]
            angular_velocity_world = [world_velocity.angular.x, world_velocity.angular.y, world_velocity.angular.z]

            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tft.euler_from_quaternion(quaternion)
            rotation_matrix = tft.euler_matrix(euler[0], euler[1], euler[2])
            linear_velocity_body = tft.inverse_matrix(rotation_matrix).dot(
                [linear_velocity_world[0], linear_velocity_world[1], linear_velocity_world[2], 1]
            )

            self.velocity.header.stamp = rospy.Time.now()
            self.velocity.twist.linear.x = linear_velocity_body[0]
            self.velocity.twist.linear.y = linear_velocity_body[1]
            self.velocity.twist.linear.z = linear_velocity_body[2]
            self.velocity.twist.angular.x = angular_velocity_world[0]
            self.velocity.twist.angular.y = angular_velocity_world[1]
            self.velocity.twist.angular.z = angular_velocity_world[2]

        except ValueError:
            rospy.logerr("Object not found in ModelStates")

    def timer_callback(self, event):
        self.velocity_pub.publish(self.velocity)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    ModelStateToTwist()
    rospy.spin()
