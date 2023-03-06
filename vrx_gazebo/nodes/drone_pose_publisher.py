#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class PoseUpdater:
    def __init__(self):
        rospy.init_node('drone_pose_publisher')
        self.pose_pub = rospy.Publisher('/vr/drone/pose', PoseStamped, queue_size=10)
        self.thruster_sub_updown = rospy.Subscriber('/drone/thrusters/updown', Float32, self.thruster_callback_updown)
        self.thruster_sub_angular = rospy.Subscriber('/drone/thrusters/angular', Float32, self.thruster_callback_angular)
        self.thruster_sub_linear = rospy.Subscriber('/drone/thrusters/linear', Float32, self.thruster_callback_linear)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = 70
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 10

    def thruster_callback_updown(self, msg):
        self.pose.pose.position.z += msg.data*0.1

    def thruster_callback_angular(self, msg):
        self.pose.pose.position.y -= msg.data
        
    def thruster_callback_linear(self, msg):
        self.pose.pose.position.x += msg.data

    def publish_pose(self):
        # set the header timestamp to the current time
        self.pose.header.stamp = rospy.Time.now()
        # publish the updated pose
        self.pose_pub.publish(self.pose)

    def run(self):
        rate = rospy.Rate(10) # publish at 10 Hz
        while not rospy.is_shutdown():
            self.publish_pose()
            rate.sleep()

if __name__ == '__main__':
    pose_updater = PoseUpdater()
    pose_updater.run()

