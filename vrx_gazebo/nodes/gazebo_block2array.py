#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray


class GazeboBlock2Array:
    def __init__(self):
        self.sub_model = rospy.Subscriber("gazebo/model_states/throttle", ModelStates, self.model_callback, queue_size=10)
        self.pub_block_array = rospy.Publisher("unity/blocks", Float32MultiArray, queue_size=10)
        self.block_string = "block"
        self.block_array = Float32MultiArray()
        
    def model_callback(self, msg: ModelStates):
        self.block_array.data = []
        for i in range(len(msg.name)):
            if (self.block_string in msg.name[i]):
                pose = (msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z)
                self.block_array.data.extend(pose)
        self.pub_block_array.publish(self.block_array)

if __name__ == "__main__":
    rospy.init_node("gazebo_block_to_array")
    gazebo_block_to_array = GazeboBlock2Array()
    rospy.spin()
