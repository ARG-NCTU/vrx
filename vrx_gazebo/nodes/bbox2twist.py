#!/usr/bin/env python
# This script convert bbox center to twist command with PID controller

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, Bool

class Node():
    def __init__(self):
        
        # Publisher
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscriber
        sub_bbox_center_cord_topic_name = rospy.get_param('~sub_bbox_center_cord_topic', '/detr_object_detection/detection_bbox_center_cord')
        self.sub_bbox_center_cord = rospy.Subscriber(sub_bbox_center_cord_topic_name, Float32MultiArray, self.cb_bbox_center, queue_size=1) #[x,y], both x and y in boundary [-1, 1]

        sub_bbox_area_topic_name = rospy.get_param('~sub_bbox_area_topic', '/detr_object_detection/detection_bbox_area')
        self.sub_bbox_area = rospy.Subscriber(sub_bbox_area_topic_name, Float32, self.cb_bbox_area, queue_size=1)

        sub_detected_topic_name = rospy.get_param('~sub_detected_topic', '/detr_object_detection/detected')
        self.sub_detected = rospy.Subscriber(sub_detected_topic_name, Bool, self.cb_bbox_detected, queue_size=1)
        
        # PID controller
        self.kp = -0.5
        self.ki = 0.0
        self.kd = 0.0
        self.error = 0.0
        self.error_sum = 0.0
        self.error_diff = 0.0
        self.prev_error = 0.0
        # self.max_error = 0.5
        # self.max_error_sum = 0.5
        # self.max_error_diff = 0.5
        self.max_twist = 0.5
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.bbox_center = [0.0, 0.0]
        self.bbox_area = 0.0
        self.bbox_detected = False
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)

    def cb_publish(self, event):
        if self.bbox_detected:
            rospy.loginfo("Bbox detected")
            if self.bbox_area > 0.003:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.prev_error = 0.0
            else:
                if abs(self.bbox_center[0]) < 0.1:
                    print("center")
                    self.twist.linear.x = 0.3
                    self.twist.angular.z = 0.0
                    self.cmd_pub.publish(self.twist)
                    self.prev_error = 0.0
                else: 
                    print("left or right")
                    self.twist.linear.x = 0.0
                    self.error = self.bbox_center[0]
                    self.error_sum += self.error
                    self.error_diff = self.error - self.prev_error
                    self.twist.angular.z = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
                    self.cmd_pub.publish(self.twist)
                    self.prev_error = self.error
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            self.prev_error = 0.0

    def cb_bbox_center(self, data):
        self.bbox_center = data.data

    def cb_bbox_area(self, data):
        self.bbox_area = data.data

    def cb_bbox_detected(self, data):
        self.bbox_detected = data.data

if __name__ == '__main__':
    rospy.init_node('bbox2twist', anonymous=True)
    node = Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

