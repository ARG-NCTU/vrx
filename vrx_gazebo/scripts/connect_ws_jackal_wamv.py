#! /usr/bin/env python3
import fix_python3_path
import roslibpy
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool


class ROSBridgeConnector:
    def __init__(self):
        rospy.init_node("connect_ws_jackal", anonymous=True)
        self.ip = rospy.get_param("~ip", '192.168.0.70')
        self.ws = rospy.get_param("~ws",'2')        
        rosbrideg_address = 'ws://' + self.ip + ':9090'
        self.client = roslibpy.Ros(host=rosbrideg_address)
        self.client.run()
        print('WS',self.ws ,'publish data to:', rosbrideg_address)    

        if self.ws == 1:
            print("ws1")
            self.pub_jackal_pose = roslibpy.Topic(self.client, "/jackal/slam_pose", "geometry_msgs/PoseStamped")
            self.pub_scan =  roslibpy.Topic(self.client, "/jackal/RL/scan", "sensor_msgs/LaserScan")
            self.pub_reset = roslibpy.Topic(self.client, "/reset", "Bool")
            self.sub_scan2_RL = rospy.Subscriber("/wamv2/RL/scan", LaserScan, self.cb_laser_1sub2_RL)
            
        elif self.ws == 2:   
            print("ws2") 
            # pose            
            self.pub_wamv2 = roslibpy.Topic(self.client, "/gazebo/wamv2/pose", "geometry_msgs/PoseStamped")
            self.pub_scan2_RL =  roslibpy.Topic(self.client, "/wamv2/RL/scan", "sensor_msgs/LaserScan")
            self.pub_jackal_goal = roslibpy.Topic(self.client, "/move_base_simple/goal", "geometry_msgs/PoseStamped")
            self.pub_position_circle1 = roslibpy.Topic(self.client, "/visualization_circle1", "visualization_msgs/Marker")
            self.pub_jackal_joy = roslibpy.Topic(self.client, "/jackal/bluetooth_teleop/joy", "sensor_msgs/Joy")
            # transform frame from wamv to wamv2
            self.sub_scan_RL = rospy.Subscriber("/jackal/RL/scan", LaserScan, self.cb_laser_2sub1_RL)
            self.pub_jackal_scan = rospy.Publisher("/jackal/RL/scan_2", LaserScan, queue_size = 1)
            

        else:
            pass
        
        self.sub_joy = rospy.Subscriber("/vr_teleop", Joy, self.cb_joy)
        self.init_subscribers()

    def init_subscribers(self):
        if self.ws == 1:
            # rospy.Subscriber("/wamv/joy", Joy, self.cb_joy)
            rospy.Subscriber("/jackal/slam_pose", PoseStamped, self.cb_jackal_pose)
            rospy.Subscriber("/jackal/RL/scan", LaserScan, self.cb_jackal_laser_RL)
            rospy.Subscriber("/reset", Bool, self.cb_reset)


        elif self.ws == 2:
            rospy.Subscriber("/gazebo/wamv2/pose", PoseStamped, self.cb_wamv2_pose)       
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_jackal_goal)
            rospy.Subscriber("/visualization_circle1", Marker, self.cb_position_circle1)  
            rospy.Subscriber("/wamv2/RL/scan", LaserScan, self.cb_wamv2_laser_RL)
            rospy.Subscriber("/jackal/bluetooth_teleop/joy", Joy, self.cb_joy_jackal)
            rospy.Subscriber("/vr_teleop", Joy, self.cb_vr_joy)
            
        else:
            pass
        
    
    def cb_joy_jackal(self, data):
        self.cb_joy(data, self.pub_jackal_joy)
        
    def cb_vr_joy(self, data):
        self.cb_joy(data, self.pub_jackal_joy)
        
    def cb_reset(self, data):
        roslib_msg = roslibpy.Message(
            {
                "data": data.data
            }
        )
        self.pub_reset.publish(roslib_msg)

    def cb_laser_1sub2_RL(self, msg):
        laser = msg
        laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "jackal/velodyne"
        self.pub_2scan_for1_RL.publish(laser)
         
    def cb_laser_2sub1_RL(self, msg):
        laser = msg
        laser.header.stamp = rospy.Time.now()
        laser.header.frame_id = "wamv2/lidar_wamv_link"
        self.pub_jackal_scan.publish(laser)
            
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
    
    def cb_wamv2_laser_RL(self, data):
        self.cb_laser(data, self.pub_scan2_RL)
        
    def cb_jackal_laser_RL(self, data):
        self.cb_laser(data, self.pub_scan_RL)

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

    def cb_jackal_pose(self, data):
        self.cb_posestamped(data, self.pub_jackal_pose)
        
    def cb_wamv2_pose(self, data):
        self.cb_posestamped(data, self.pub_wamv2)
        
    def cb_jackal_goal(self, data):
        self.cb_posestamped(data, self.pub_jackal_goal)
            
if __name__ == '__main__':
    connector = ROSBridgeConnector()
    while not rospy.is_shutdown():
        rospy.spin()