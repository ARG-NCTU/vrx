#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DrawPath():
    def __init__(self):
        
        # Subscriber
        sub_wamv_pose_topic_name = rospy.get_param('~sub_wamv_pose_topic', '/gazebo/wamv2/pose')
        self.sub_wamv = rospy.Subscriber(sub_wamv_pose_topic_name, PoseStamped, self.cb_wamv, queue_size=1)
        
        # Publisher
        self.pub_wamv_path = rospy.Publisher('wamv_path', Path, queue_size=1)

        self.wamv_path = Path()

    def cb_wamv(self, pose):
        self.wamv_path.header.frame_id = 'map'
        self.wamv_path.header.stamp = rospy.Time.now()
        self.wamv_path.poses.append(pose)
        self.pub_wamv_path.publish(self.wamv_path)


if __name__=="__main__":
    rospy.init_node("draw_path", anonymous=True)
    draw_path = DrawPath()
    rospy.spin()