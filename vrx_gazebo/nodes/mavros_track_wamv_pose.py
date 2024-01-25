#! /usr/bin/env python
import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MavrosTrackWAMVPose:
    FAIL = 0
    RUNNING = 1
    SUCCESS = 2

    def __init__(self):
        self.wamv_pose_to_local_sub = rospy.Subscriber(
            "/wamv/localization_gps_imu/pose", PoseStamped, self.wamv_pose_to_local_callback
        )
        self.setpoint_position_local_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        node_name = rospy.get_name().split("/")[-1]
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)

        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.wamv_pose_to_local = PoseStamped()
        self.wamv_pose_to_local_received = False

        self.active = Active()

    def wamv_pose_to_local_callback(self, msg):
        self.wamv_pose_to_local = msg
        self.wamv_pose_to_local_received = True

    def behavior_active_callback(self, msg):
        self.active = msg

    def timer_callback(self, event):
        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = self.RUNNING
            self.behavior_status_pub.publish(status)
            self.wamv_pose_to_local.pose.position.z += 2.0
            self.wamv_pose_to_local.pose.position.x -= 5.0

            self.setpoint_position_local_pub.publish(self.wamv_pose_to_local)
        else:
            status = Status()
            status.id = self.active.id
            status.status = self.FAIL
            self.behavior_status_pub.publish(status)
        rospy.loginfo("active: {}".format(self.active.active))
        rospy.loginfo("status: {}".format(self.active.id))


if __name__ == "__main__":
    rospy.init_node("mavros_track_wamv_pose")
    mavros_track_wamv_gps = MavrosTrackWAMVPose()
    rospy.spin()
