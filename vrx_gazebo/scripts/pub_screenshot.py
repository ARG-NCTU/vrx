#!/usr/bin/env python3
import rospy
import rospkg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import pyautogui
import cv2
import numpy as np


global start_x, start_y, end_x, end_y
global flag
start_x, start_y, end_x, end_y = 260, 60, 745, 490
flag = False
rospy.init_node("screenshot", anonymous=True)
# pub = rospy.Publisher('mini_map', Image, queue_size=10)
pub = rospy.Publisher('mini_map', CompressedImage, queue_size=10)
bridge = CvBridge()

def show_xy(event,x,y,flags,userdata):
	global start_x, start_y, end_x, end_y
	global flag
	if event==1:
		flag = True
		start_x, start_y = x, y
		print("from: x,y=", start_x, start_y)
	if event==4 or event==0 :
		if flag :
			end_x, end_y = x, y
			print("to: x,y=", end_x, end_y)
			if event==4 : flag = False

# rospy.spin()
while True:
	myScreenshot = pyautogui.screenshot()
	image = np.asarray(myScreenshot)
	image = cv2.resize(image, (1920, int(image.shape[0]*(1920/image.shape[1])) ), interpolation=cv2.INTER_AREA)

	if start_x>0 and  start_y>0 and  end_x>0 and end_y>0:
		_x1,_y1 = min(start_x,end_x), min(start_y,end_y)
		_x2,_y2 = max(start_x,end_x), max(start_y,end_y)
		crop_img = image[_y1:_y2, _x1:_x2]
		# cv2.rectangle(image, (_x1,_y1), (_x2,_y2), (0,255,0),1)
		
		try:
			pub.publish(bridge.cv2_to_compressed_imgmsg(crop_img[:,:,::-1]))
		except :
			rospy.loginfo("Invalid CV bridge")

	cv2.imshow('screenshot',image)
	cv2.setMouseCallback('screenshot', show_xy)  # 設定偵測事件的函式與視窗
	rospy.sleep(0.4)



	if cv2.waitKey(1) == ord('q'):
		break
cv2.destroyAllWindows()


# install `pip3 install pyautogui` `sudo apt-get install scrot python3-tk python3-dev`
# env `export GDK_SYNCHRONIZE=1`