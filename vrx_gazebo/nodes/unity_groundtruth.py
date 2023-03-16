#!/usr/bin/env python3

import csv
import cv2
import math
import numpy as np

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray

DIVIDER = '--------------------------'

class Unity_Groundtruth():
    def __init__(self):
        self.line_thickness = 5
        self.gt_bbox = None
        self.agent_pose = [None] * 4
        self.gtbbox_recieved = False
        # Subscriber
        self.gt_bbox_sub = rospy.Subscriber("unity/gt_bbox", Float32MultiArray, self.sub_gt_bbox, queue_size=1)
        self.image_sub = rospy.Subscriber("unity/camera_under/compressed", CompressedImage, self.sub_img, queue_size=1)
        # Publisher
        self.pub_target_screen = rospy.Publisher("unity/gt_img/compressed", CompressedImage, queue_size=1)

        # Show some informations
        rospy.loginfo("Groundtruth init done.")
    
    def sub_img(self, data):
        self.last_image = data
        #rospy.loginfo("sub_img")
    
    def crop_ROI(self, img, bbox):
        np_arr = np.fromstring(img.data, np.uint8)
        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        xmin = int(bbox[0])
        ymin = int(bbox[1])
        xmax = int(bbox[2])
        ymax = int(bbox[3])
        roi = img_np[ymin:ymax, xmin:xmax]
        #rospy.loginfo(roi)
        roi = np.array(cv2.imencode('.jpg', roi)[1]).tostring()
        return roi

    def sub_gt_bbox(self, msg):
        self.gt_bbox = msg.data
        self.gtbbox_recieved = True
        rospy.loginfo("sub_gt_bbox")
        input_img = self.last_image
        if self.gt_bbox[0] <= 4:
            class_name = 'fish'
        else:
            class_name = 'boat'
        # self.total_label[class_name] += 1
        xmin = self.gt_bbox[2]
        ymin = self.gt_bbox[3]
        xmax = self.gt_bbox[4]
        ymax = self.gt_bbox[5]
        gt_bbox_label = self.gt_bbox[1]
        rospy.loginfo("Click")
        roi = self.crop_ROI(input_img, [int(xmin), int(ymin), int(xmax), int(ymax)])

        # Publish ROI result
        rospy.loginfo("Publish")
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', roi)[1]).tostring()
        msg.data = roi
        self.pub_target_screen.publish(msg)

if __name__=="__main__":
    rospy.init_node("unity_groundtruth", anonymous=True)
    unity_groundtruth = Unity_Groundtruth()
    rospy.spin()
