#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class DroneTargetDisChecker:
	def __init__(self):
		self.sub_arm_target = rospy.Subscriber("/gazebo/target/pose", PoseStamped, self.target_callback)
		self.sub_arm = rospy.Subscriber("/gazebo/drone/pose", PoseStamped, self.arm_callback)
		self.pub_checker = rospy.Publisher("/checker", Bool, queue_size=1)
		self.pub_checker.publish(False)
		self.timer = rospy.Timer(rospy.Duration(0.1), self.check_distance)
		self.arm_target_pose = PoseStamped()
		self.arm_pose = PoseStamped()
		self.dis_limit = 1.0

	def target_callback(self, msg):
		self.arm_target_pose = msg
	
	def arm_callback(self, msg):
		self.arm_pose = msg
		
	def check_distance(self,event):
		dis = math.sqrt((self.arm_target_pose.pose.position.x - self.arm_pose.pose.position.x)**2 + (self.arm_target_pose.pose.position.y - self.arm_pose.pose.position.y)**2)
		if dis < self.dis_limit:
			self.pub_checker.publish(True)
		else:
			self.pub_checker.publish(False)
if __name__ == "__main__":
	rospy.init_node("drone_target_dis_checker")
	DroneTargetDisChecker()
	rospy.spin()