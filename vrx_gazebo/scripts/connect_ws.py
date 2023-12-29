#! /usr/bin/env python3
import fix_python3_path
import roslibpy
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
# from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker

class ROSBridgeConnector:
    def __init__(self):
        rospy.init_node("connect_ws", anonymous=True)
        self.ip = rospy.get_param("~ip", '127.0.0.1')
        self.ws = rospy.get_param("~ws",'1')
        self.vr = rospy.get_param("~vr",'true')
        
        rosbrideg_address = 'ws://' + self.ip + ':9090'
        print('WS',self.ws ,'publish data to:', rosbrideg_address)    
        self.client = roslibpy.Ros(host=rosbrideg_address)
        self.client.run()


        if self.ws == 1:
            print("ws1")
            self.pub_wamv_pose = roslibpy.Topic(self.client, "/gazebo/wamv/pose", "geometry_msgs/PoseStamped")
            # self.pub_joy = roslibpy.Topic(self.client, "/wamv/joy", "sensor_msgs/Joy")
            self.pub_scan =  roslibpy.Topic(self.client, "/wamv/RL/more_scan", "sensor_msgs/LaserScan")
            # self.pub_obstacle_extractor = roslibpy.Topic(self.client, "/raw_obstacles", "obstacle_detector/Obstacles")
            self.sub_scan2 = rospy.Subscriber("/wamv2/RL/more_scan", LaserScan, self.cb_laser_1sub2)
            self.pub_2scan_for1 = rospy.Publisher("/wamv2/RL/more_scan_2", LaserScan, queue_size=1)
            
            # if self.vr == False:
                # self.pub_joy = roslibpy.Topic(self.client, "/joy", "sensor_msgs/Joy")
                # print('joy : ws1->ws2')

        elif self.ws == 2:   
            print("ws2")             
            self.pub_fake_pose = roslibpy.Topic(self.client, "/fake_fence_real2sim", "geometry_msgs/PoseStamped")
            self.pub_wamv2 = roslibpy.Topic(self.client, "/gazebo/wamv2/pose", "geometry_msgs/PoseStamped")
            # self.pub_cmd = roslibpy.Topic(self.client, "/wamv/cmd_vel", "Twist")
            self.pub_scan2 =  roslibpy.Topic(self.client, "/wamv2/RL/more_scan", "sensor_msgs/LaserScan")
            self.pub_wamv_goal = roslibpy.Topic(self.client, "/wamv/move_base_simple/goal", "geometry_msgs/PoseStamped")
            self.pub_position_circle1 = roslibpy.Topic(self.client, "/visualization_circle1", "visualization_msgs/Marker")
            
            self.sub_scan = rospy.Subscriber("/wamv/RL/more_scan", LaserScan, self.cb_laser_2sub1)
            self.pub_1scan_for2 = rospy.Publisher("/wamv/RL/more_scan_2", LaserScan, queue_size=1)
            # if self.vr == True :
            self.pub_joy_wamv = roslibpy.Topic(self.client, "/wamv/joy", "sensor_msgs/Joy")
            self.pub_joy = roslibpy.Topic(self.client, "/joy", "sensor_msgs/Joy")

            self.pub_obstacle_to_wamv = roslibpy.Topic(self.client, "/obstacle_from_real", "geometry_msgs/PoseStamped")

            # if self.vr == False: 
            #     self.pub_joy = roslibpy.Topic(self.client, "/joy", "sensor_msgs/Joy")
            print('joy : ws2-> ws1')
        else:
            pass

        self.init_subscribers()


    def init_subscribers(self):
        if self.ws == 1:
            rospy.Subscriber("/gazebo/wamv/pose", PoseStamped, self.cb_wamv_pose)
            # rospy.Subscriber("/wamv/joy", Joy, self.cb_joy)
            # rospy.Subscriber("/raw_obstacles", Obstacles, self.cb_extractor)
            rospy.Subscriber("/wamv/RL/more_scan", LaserScan, self.cb_wamv_laser)
            # if self.vr == False:
            #     rospy.Subscriber("/joy", Joy, self.cb_joy)
            
        elif self.ws == 2:
            rospy.Subscriber("/fake_fence_real2sim", PoseStamped, self.cb_fake_pose)
            # rospy.Subscriber("/wamv/cmd_vel", Twist, self.cb_twist)
            rospy.Subscriber("/gazebo/wamv2/pose", PoseStamped, self.cb_wamv2_pose)
            rospy.Subscriber("/wamv/move_base_simple/goal", PoseStamped, self.cb_wamv_goal)
            rospy.Subscriber("/visualization_circle1", Marker, self.cb_position_circle1)
            rospy.Subscriber("/wamv2/RL/more_scan", LaserScan, self.cb_wamv2_laser)
            rospy.Subscriber("/obstacle_from_real", PoseStamped, self.cb_obstacle_to_wamv)
            # if self.vr == True :
            rospy.Subscriber("/wamv/joy", Joy, self.cb_joy_wamv)
            # if self.vr == False:
            rospy.Subscriber("/joy", Joy, self.cb_joy_all)
                
        else:
            pass
        
    def cb_laser_2sub1(self, msg):
        laser = msg
        laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "wamv2/lidar_wamv_link"
        self.pub_1scan_for2.publish(laser)
    
    def cb_laser_1sub2(self, msg):
        laser = msg
        laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "wamv/lidar_wamv_link"
        self.pub_2scan_for1.publish(laser)

    def cb_obstacle_to_wamv(self, data):
        self.cb_posestamped(data, self.pub_obstacle_to_wamv)

    def cb_laser(self, data, publisher):
        
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
        
        publisher.publish(roslib_msg)
        # self.pub_scan.publish(roslib_msg)
    
    def cb_wamv_laser(self, data):
        self.cb_laser(data, self.pub_scan)
        
    def cb_wamv2_laser(self, data):
        self.cb_laser(data, self.pub_scan2)
    

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

    def cb_joy(self, data, publisher):
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

        publisher.publish(roslib_msg)
        
    def cb_joy_wamv(self, data):
        self.cb_joy(data, self.pub_joy_wamv)
        
    def cb_joy_all(self, data):
        self.cb_joy(data, self.pub_joy)
        
    def cb_position_circle1(self, data):

        roslib_msg = roslibpy.Message({
            "header": {
                "seq": data.header.seq,
                "stamp": {"secs": data.header.stamp.secs, "nsecs": data.header.stamp.nsecs},
                "frame_id": data.header.frame_id
            },
            "ns": data.ns,
            "id": data.id,
            "type": data.type,
            "action": data.action,
            "pose": {
                "position": {"x": data.pose.position.x,"y": data.pose.position.y,"z": data.pose.position.z},
                "orientation": {"x": data.pose.orientation.x,"y": data.pose.orientation.y,"z": data.pose.orientation.z,"w": data.pose.orientation.w}
            },
            "scale": {"x": data.scale.x,"y": data.scale.y,"z": data.scale.z},
            "color": {"r": data.color.r,"g": data.color.g,"b": data.color.b,"a": data.color.a},
            "lifetime": {"secs": data.lifetime.secs, "nsecs": data.lifetime.nsecs},
            "frame_locked": data.frame_locked,
            "points": [{"x":point.x, "y":point.y, "z":point.z} for point in data.points],
            "colors": data.colors,
            "text": data.text,
            "mesh_resource": data.mesh_resource,
            "mesh_use_embedded_materials": data.mesh_use_embedded_materials
        })
        self.pub_position_circle1.publish(roslib_msg)

    def cb_wamv_pose(self, data):
        self.cb_posestamped(data, self.pub_wamv_pose)

    def cb_fake_pose(self, data):
        self.cb_posestamped(data, self.pub_fake_pose)
        
    def cb_wamv2_pose(self, data):
        self.cb_posestamped(data, self.pub_wamv2)
        
    def cb_wamv_goal(self, data):
        self.cb_posestamped(data, self.pub_wamv_goal)
            
if __name__ == '__main__':
    connector = ROSBridgeConnector()
    while not rospy.is_shutdown():
        rospy.spin()