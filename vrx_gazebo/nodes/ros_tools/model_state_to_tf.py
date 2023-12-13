#! /usr/bin/env python3

import fix_python3_path
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class ModelStateToTf:
    def __init__(self):
        rospy.init_node("model_state_to_tf")

        self.model_name = rospy.get_param("~model_name", "wamv")
        self.model_tf_suffix = rospy.get_param("~model_tf_suffix", "base_link")
        self.brocast_rate = rospy.get_param("~broadcast_rate", 100)

        self.child_frame_id = self.model_name + ("/" if not self.model_tf_suffix.startswith("/") else "") + self.model_tf_suffix
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.br_timer = rospy.Timer(rospy.Duration(1.0 / self.brocast_rate), self.broadcast_tf)

        self.tf_msg = TransformStamped()

    def model_callback(self, msg):
        # Get the transform from the world to the robot
        try:
            model_index = msg.name.index(self.model_name)
            rospy.loginfo_throttle(1, f"Model name: {msg.name[model_index]}")

            # Create the transform from the world to the model
            self.tf_msg.header.stamp = rospy.Time.now()
            self.tf_msg.header.frame_id = "map"
            self.tf_msg.child_frame_id = self.child_frame_id
            self.tf_msg.transform.translation = msg.pose[model_index].position
            self.tf_msg.transform.rotation = msg.pose[model_index].orientation
        except ValueError:
            rospy.logwarn_throttle(1, f"Model name {self.model_name} not found in model states message")

    def broadcast_tf(self, event):
        if self.tf_msg.header.frame_id == "":
            rospy.logwarn_throttle(1, "No transform received yet")
            return
        rospy.loginfo_throttle(
            1, f"Broadcasting transform from {self.tf_msg.header.frame_id} to {self.tf_msg.child_frame_id}"
        )
        self.tf_broadcaster.sendTransform(self.tf_msg)


if __name__ == "__main__":
    ModelStateToTf()
    rospy.spin()
