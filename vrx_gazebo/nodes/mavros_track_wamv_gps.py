#! /usr/bin/env python
import rospy
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler


class MavrosTrackWAMVGPS:
    def __init__(self):
        self.gps_sub = rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback)
        self.geo_pose_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
    
    def gps_callback(self, msg):
        rospy.loginfo_throttle(1.0, 'Latitude: %f, Longitude: %f', msg.latitude, msg.longitude)
        geo_pose_msg = GeoPoseStamped()
        geo_pose_msg.header.stamp = rospy.Time.now()
        geo_pose_msg.header.frame_id = 'world'
        geo_pose_msg.pose.position.latitude = msg.latitude
        geo_pose_msg.pose.position.longitude = msg.longitude
        geo_pose_msg.pose.position.altitude = 10.0
        q = quaternion_from_euler(0.0, 0.0, 45.0)
        geo_pose_msg.pose.orientation.x = q[0]
        geo_pose_msg.pose.orientation.y = q[1]
        geo_pose_msg.pose.orientation.z = q[2]
        geo_pose_msg.pose.orientation.w = q[3]
        self.geo_pose_pub.publish(geo_pose_msg)
    
if __name__ == '__main__':
    rospy.init_node('mavros_track_wamv_gps')
    mavros_track_wamv_gps = MavrosTrackWAMVGPS()
    rospy.spin()
