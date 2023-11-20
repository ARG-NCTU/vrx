#! /usr/bin/env python3
import roslibpy
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
from obstacle_detector.msg import Obstacles

class ROSBridgeConnector:
    def __init__(self):
        rospy.init_node("connect_ws", anonymous=True)
        self.ip = rospy.get_param("~ip", '127.0.0.1')
        self.ws = rospy.get_param("~ws",'1')
        rosbrideg_address = 'ws://' + self.ip + ':9090'
        print('WS',self.ws ,'publish data to:', rosbrideg_address)    
        self.client = roslibpy.Ros(host=rosbrideg_address)
        self.client.run()

        if self.ws == 1:
            print("ws1")
            self.pub_wamv_pose = roslibpy.Topic(self.client, "/wamv/truth_map_posestamped", "geometry_msgs/PoseStamped")
            self.pub_joy = roslibpy.Topic(self.client, "/joy", "sensor_msgs/Joy")
            self.pub_scan =  roslibpy.Topic(self.client, "/wamv/RL/more_scan", "sensor_msgs/LaserScan")

            # self.pub_obstacle_extractor = roslibpy.Topic(self.client, "/raw_obstacles", "obstacle_detector/Obstacles")
        elif self.ws == 2:   
            print("ws2")             
            self.pub_fake_pose = roslibpy.Topic(self.client, "/fake_fence_real2sim", "geometry_msgs/PoseStamped")
            self.pub_wamv2 = roslibpy.Topic(self.client, "/wamv2/truth_map_posestamped", "geometry_msgs/PoseStamped")
            self.pub_cmd = roslibpy.Topic(self.client, "/wamv/cmd_vel", "Twist")
        else:
            pass
        self.init_subscribers()

    def init_subscribers(self):
        if self.ws == 1:
            rospy.Subscriber("/wamv/truth_map_posestamped", PoseStamped, self.cb_wamv_pose)
            rospy.Subscriber("/joy", Joy, self.cb_joy)
            # rospy.Subscriber("/raw_obstacles", Obstacles, self.cb_extractor)
            rospy.Subscriber("/wamv/RL/more_scan", LaserScan, self.cb_laser)
        elif self.ws == 2:
            rospy.Subscriber("/fake_fence_real2sim", PoseStamped, self.cb_fake_pose)
            rospy.Subscriber("/wamv/cmd_vel", Twist, self.cb_twist)
            rospy.Subscriber("/wamv2/truth_map_posestamped", PoseStamped, self.cb_wamv2_pose)
        else:
            pass
        
    def cb_laser(self, data):
        
        roslib_msg = roslibpy.Message(
            {   "header": {
                    "seq": data.header.seq,
                    "stamp": {"secs": data.header.stamp.secs, "nsecs": data.header.stamp.nsecs},
                    "frame_id": data.header.frame_id
                },
                "angle_min": data.angle_min,
                "angle_max": data.angle_max,
                "angle_increment": data.angle_increment,
                "time_increment": data.time_increment,
                "scan_time": data.scan_time,
                "range_min": data.range_min,
                "range_max": data.range_max,
                "ranges": data.ranges,
                "intensities": data.intensities
            }
        )
        
        self.pub_scan.publish(roslib_msg)
        
    def cb_posestamped(self, data, publisher):
        roslib_msg = roslibpy.Message(
            {
                "header": {
                    "seq": data.header.seq,
                    "stamp": {"secs": data.header.stamp.secs, "nsecs": data.header.stamp.nsecs},
                    "frame_id": data.header.frame_id
                },
                "pose": {
                    "position": {"x": data.pose.position.x, "y": data.pose.position.y, "z": data.pose.position.z},
                    "orientation": {"x": data.pose.orientation.x, "y": data.pose.orientation.y, "z": data.pose.orientation.z, "w": data.pose.orientation.w}
                }
            }
        )
        publisher.publish(roslib_msg)

    def cb_extractor(self, data):

        roslib_msg = roslibpy.Message(
            {
            "header": {
                "stamp": {"secs": data.header.stamp.secs, "nsecs": data.header.stamp.nsecs},
                "frame_id": data.header.frame_id,
                "seq": data.header.seq,
                },
        "circles": 
            [{
                "center": {"x": circle.center.x, "y": circle.center.y, "z": circle.center.z},
                "velocity": {"x": circle.velocity.x, "y": circle.velocity.y, "z": circle.velocity.z},
                "radius": circle.radius,
                "true_radius": circle.true_radius,
            } for circle in data.circles]
        })
        # rospy.loginfo(roslib_msg)
        self.pub_obstacle_extractor.publish(roslib_msg)

        
    def cb_twist(self, data):
        roslib_msg = roslibpy.Message(
            {
                "linear": {"x": data.linear.x, "y": data.linear.y, "z": data.linear.z},
                "angular": {"x": data.angular.x, "y": data.angular.y, "z": data.angular.z}
            }
        )
        self.pub_cmd.publish(roslib_msg)

    def cb_joy(self, data):
        roslib_msg = roslibpy.Message(
            {
                "header": {
                    "seq": data.header.seq,
                    "stamp": {"secs": data.header.stamp.secs, "nsecs": data.header.stamp.nsecs},
                    "frame_id": data.header.frame_id
                },
                "axes": data.axes,
                "buttons": data.buttons
            }
        )
        self.pub_joy.publish(roslib_msg)

    def cb_wamv_pose(self, data):
        self.cb_posestamped(data, self.pub_wamv_pose)

    def cb_fake_pose(self, data):
        self.cb_posestamped(data, self.pub_fake_pose)
        
    def cb_wamv2_pose(self, data):
        self.cb_posestamped(data, self.pub_wamv2)
        
if __name__ == '__main__':
    connector = ROSBridgeConnector()
    while not rospy.is_shutdown():
        rospy.spin()
