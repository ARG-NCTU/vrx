#! /usr/bin/env python
import rospy
from behavior_tree_msgs.msg import Active, Status
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class MavrosTrackWAMVGPS:
    FAIL = 0
    RUNNING = 1
    SUCCESS = 2

    def __init__(self):
        self.trigger_timeout = rospy.get_param("~trigger_timeout", 1.0)
        self.trigger_sub = rospy.Subscriber("/track_trigger", Bool, self.trigger_callback)
        self.gps_sub = rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.gps_callback)
        self.geo_pose_pub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)
        node_name = rospy.get_name().split("/")[-1]
        print(node_name)

        self.behavior_active_sub = rospy.Subscriber("/" + node_name + "_active", Active, self.behavior_active_callback)
        self.behavior_status_pub = rospy.Publisher("/" + node_name + "_status", Status, queue_size=1)

        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.active = Active()
        self.trigger = True
        self.trigger_time = rospy.Time.now()

    def trigger_callback(self, msg):
        self.trigger = msg.data
        self.trigger_time = rospy.Time.now()

    def gps_callback(self, msg):
        rospy.loginfo_throttle(1.0, "Latitude: %f, Longitude: %f", msg.latitude, msg.longitude)
        geo_pose_msg = GeoPoseStamped()
        geo_pose_msg.header.stamp = rospy.Time.now()
        geo_pose_msg.header.frame_id = "world"
        geo_pose_msg.pose.position.latitude = msg.latitude
        geo_pose_msg.pose.position.longitude = msg.longitude - 4.0e-5
        geo_pose_msg.pose.position.altitude = 10.0
        q = quaternion_from_euler(0.0, 0.0, 0.0)

        geo_pose_msg.pose.orientation.x = q[0]
        geo_pose_msg.pose.orientation.y = q[1]
        geo_pose_msg.pose.orientation.z = q[2]
        geo_pose_msg.pose.orientation.w = q[3]
        self.geo_pose_pub.publish(geo_pose_msg)

    def behavior_active_callback(self, msg):
        self.active = msg

    def timer_callback(self, event):
        current_time = rospy.Time.now()

        if not self.trigger:
            return

        if current_time - self.trigger_time > rospy.Duration(self.trigger_timeout):
            self.trigger = False
            return

        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = self.RUNNING
            self.behavior_status_pub.publish(status)
        else:
            status = Status()
            status.id = self.active.id
            status.status = self.FAIL
            self.behavior_status_pub.publish(status)
        rospy.loginfo("active: {}".format(self.active.active))
        rospy.loginfo("status: {}".format(self.active.id))


if __name__ == "__main__":
    rospy.init_node("mavros_track_wamv_gps")
    mavros_track_wamv_gps = MavrosTrackWAMVGPS()
    rospy.spin()
