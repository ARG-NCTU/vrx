#! /usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tft
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class MavrosTrackWAMVUWBPose:
    FAIL = 0
    RUNNING = 1
    SUCCESS = 2

    def __init__(self):
        self.trigger_timeout = rospy.get_param("~trigger_timeout", 1.0)

        self.trigger_sub = rospy.Subscriber("/track_trigger", Bool, self.trigger_callback)

        self.drone_pose_to_wamv_sub = rospy.Subscriber(
            "/pozyx_simulation/drone/pose/optim", PoseStamped, self.drone_pose_to_wamv_callback
        )
        self.drone_pose_to_local_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )
        self.setpoint_position_local_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

        node_name = rospy.get_name().split("/")[-1]
        print(node_name)

        self.behavior_active_sub = rospy.Subscriber("/" + node_name + "_active", Active, self.behavior_active_callback)
        self.behavior_status_pub = rospy.Publisher("/" + node_name + "_status", Status, queue_size=1)

        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.active = Active()

        self.trigger = True

        self.drone_pose_to_wamv = np.identity(4)
        self.drone_pose_to_local = np.identity(4)

        self.trigger_time = rospy.Time.now()

    def trigger_callback(self, msg):
        self.trigger = msg.data
        self.trigger_time = rospy.Time.now()

    def drone_pose_to_wamv_callback(self, msg):
        self.drone_pose_to_wamv = self.pose_stamped_to_matrix(msg)

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose_to_local = self.pose_stamped_to_matrix(msg)

    def behavior_active_callback(self, msg):
        self.active = msg

    def timer_callback(self, event):
        current_time = rospy.Time.now()

        # if not self.trigger:
        #     return

        # if current_time - self.trigger_time > rospy.Duration(self.trigger_timeout):
        #     self.trigger = False
        #     return

        if not np.allclose(self.drone_pose_to_wamv, np.identity(4)):
            if not np.allclose(self.drone_pose_to_local, np.identity(4)):
                wamv_to_drone = np.linalg.inv(self.drone_pose_to_wamv)
                wamv_to_drone[0, 3] += -1.5
                wamv_to_drone[2, 3] += 1.0
                wamv_to_local = np.dot(self.drone_pose_to_local, wamv_to_drone)
                pose_stamped = self.matrix_to_pose_stamped(wamv_to_local, "map")
                # pose_stamped.pose.position.z = 0.0
                if 1 or self.active.active:
                    self.setpoint_position_local_pub.publish(pose_stamped)
                # rospy.loginfo_throttle(1.0, "WAMV to local: \n%s", wamv_to_local)
                # rospy.loginfo_throttle(1.0, "WAMV to local: \n%s", pose_stamped)
            else:
                rospy.loginfo_throttle(1.0, "Drone pose to local is not available.")
        else:
            rospy.loginfo_throttle(1.0, "Drone pose to WAMV is not available.")

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

    def pose_stamped_to_matrix(self, pose):
        matrix = tft.quaternion_matrix(
            (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        )
        matrix[0, 3] = pose.pose.position.x
        matrix[1, 3] = pose.pose.position.y
        matrix[2, 3] = pose.pose.position.z
        return matrix

    def matrix_to_pose_stamped(self, matrix, frame_id):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]
        q = tft.quaternion_from_matrix(matrix)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose


if __name__ == "__main__":
    rospy.init_node("mavros_track_wamv_uwb_pose")
    mavros_track_wamv_gps = MavrosTrackWAMVUWBPose()
    rospy.spin()
