#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class DroneArmedPose:
    def __init__(self):
        # Subscribers
        self.pose_sub = rospy.Subscriber('/gazebo/drone/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

        # Publisher
        self.pose_pub = rospy.Publisher('/drone/armed_pose', PoseStamped, queue_size=10)

        self.drone_armed = False
        self.initial_pose_recorded = False
        self.initial_pose = None

        rospy.spin()

    def pose_callback(self, data):
        if self.drone_armed and not self.initial_pose_recorded:
            self.initial_pose = data
            self.initial_pose_recorded = True
            rospy.loginfo("Initial pose recorded as origin location: %s", self.initial_pose)

        if self.initial_pose_recorded:
            self.pose_pub.publish(self.initial_pose)

    def state_callback(self, data):
        self.drone_armed = data.armed
        if self.drone_armed:
            rospy.loginfo("Drone is armed")
        else:
            rospy.loginfo("Drone is disarmed")
            self.initial_pose_recorded = False

if __name__ == '__main__':
    try:
        rospy.init_node('drone_armed_pose', anonymous=True)
        DroneArmedPose()
    except rospy.ROSInterruptException:
        pass