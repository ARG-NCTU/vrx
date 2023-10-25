#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
import csv
import os 
import time
from nav_msgs.msg import Path

class RecordTraj(object):
    def __init__(self):
        
        self.wamv_x, self.wamv_y, self.wamv2_x, self.wamv2_y, self.fake_x, self.fake_y = 0, 0, 0, 0, 0, 0
        self.wamv_orientation = [0, 0, 0, 0]
        self.wamv2_orientation = [0, 0, 0, 0]
        self.fake_orientation = [0, 0, 0, 0]
        self.linear_x, self.linear_y, self.linear_z, self.angular_x, self.angular_y, self.angular_z = 0, 0, 0, 0, 0, 0
        
        self.sub_wamv1 = rospy.Subscriber("/wamv/truth_map_posestamped", PoseStamped, self.cb_wamv, queue_size=1)
        self.sub_wamv2 = rospy.Subscriber("/wamv2/truth_map_posestamped", PoseStamped, self.cb_wamv2, queue_size=1)
        self.sub_fake_pose = rospy.Subscriber("/fake_fence_real2sim", PoseStamped, self.cb_fake_pose, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)
        self.sub_vel = rospy.Subscriber("/wamv/cmd_vel", Twist, self.cb_vel, queue_size=1)
        self.flag = False
        self.update = True
        self.create_csv()
        self.cnt = 0
        self.joy = [0, 0]
        self.vel = [0, 0, 0, 0, 0, 0]
        self.stop_record = 0
        
        self.pub_path_wamv = rospy.Publisher('/wamv/trajectory', Path, queue_size=1)
        self.pub_path_wamv2 = rospy.Publisher('/wamv2/trajectory', Path, queue_size=1)
        self.pub_path_fake = rospy.Publisher('/fake/trajectory', Path, queue_size=1)
        
    def create_csv(self):
        # self.file_path = os.path.expanduser('~/robotx-2022/csv/wamv_pose.csv')
        date = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.file_path = '/home/argrobotx/robotx-2022/catkin_ws/src/vrx/vrx_gazebo/scripts/fake_obstacle/csv/' +  date + '.csv'
        
        rospy.loginfo(self.file_path)
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        rospy.loginfo('create csv')   
        with open(self.file_path, 'w', newline='') as csvfile:
            self.writer = csv.writer(csvfile)
            self.writer.writerow(['count','wamv_x', 'wamv_y', 'wamv_orientation',
                                  'wamv2_x','wamv2_y','wamv2_orientation',
                                  'fake_x', 'fake_y', 'fake_orientation',
                                  'joy_auto/manual', 'vel'])

            
    def record_pose (self):
        self.time = rospy.get_time()/1000000000
        if self.flag == True:
            with open(self.file_path, 'a', newline='') as csvfile:
                self.writer = csv.writer(csvfile)

                self.writer.writerow([self.cnt, self.wamv_x, self.wamv_y, self.wamv_orientation,
                                     self.wamv2_x, self.wamv2_y, self.wamv2_orientation,
                                     self.fake_x, self.fake_y, self.fake_orientation, 
                                     self.joy, self.vel])
                self.cnt += 1
              
        if self.flag == False :
            pass
        
    def cb_poststamp(self, msg, sub=None):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = msg.header.frame_id  
        if sub == 'wamv':
            self.wamv_x = msg.pose.position.x
            self.wamv_y = msg.pose.position.y
            self.wamv_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        elif sub == 'wamv2':
            self.wamv2_x = msg.pose.position.x
            self.wamv2_y = msg.pose.position.y
            self.wamv2_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        elif sub == 'fake':
            self.fake_x = msg.pose.position.x
            self.fake_y = msg.pose.position.y
            self.fake_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
          
    def cb_wamv(self, msg):
        self.cb_poststamp(msg,sub='wamv')
        
    def cb_wamv2(self, msg):
        self.cb_poststamp(msg,sub='wamv2')    
        
    def cb_fake_pose(self, msg):
        self.cb_poststamp(msg,sub='fake')
        
    def cb_vel(self, msg):
        linear_x = msg.linear.x
        liner_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        self.vel = [linear_x, liner_y,linear_z, angular_x, angular_y, angular_z]
        
    def cb_joy(self, msg):
        start_btn = 5  
        end_btn = 4 
        self.joy = [msg.buttons[7], msg.buttons[6]] # auto/manual
        self.stop_record = msg.buttons[4]
        if (msg.buttons[start_btn] == 1 ) and self.update:
            self.update = False
            self.flag = True 
            rospy.loginfo('start record')
        if(msg.buttons[end_btn] == 1 ) and not self.update:
            self.update = True

        if (msg.buttons[7] == 1 or msg.buttons[6] == 1):
            self.cnt = 0   
            rospy.loginfo('switch mode')
            
    # def pub_trajectory(self):
    #     path_msg = Path()
    #     path_msg.header.frame_id = 'map'
    #     pose1 = PoseStamped()
    #     pose1.header.frame_id = "map"  # Set the same frame ID as the Path
    #     pose1.pose.position.x = self.wamv_x
    #     pose1.pose.position.y = self.wamv_y
    #     pose1.pose.position.z = self.wamv_z
    #     path_msg.poses.append(self.wamv_x)
        
        
    def run(self): 
        while not rospy.is_shutdown():
            self.record_pose()
            rospy.sleep(0.1)  
            if (self.stop_record == 1) and self.update: 
                rospy.loginfo('stop record')
                rospy.loginfo('save file in %s', self.file_path)
                break  # Exit the loop


if __name__ == '__main__':
    rospy.init_node('Record_Traj')
    record_traj = RecordTraj()
    try:
        record_traj.run()
    except rospy.ROSInterruptException:
        pass