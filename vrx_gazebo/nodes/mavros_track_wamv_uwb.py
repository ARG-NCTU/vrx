#! /usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped


class MavrosTrackWAMVUWBPose:
    def __init__(self):
        self.drone_pose_to_wamv_sub = rospy.Subscriber(
            "/pozyx_simulation/drone/pose/optim", PoseStamped, self.drone_pose_to_wamv_callback
        )
        self.drone_pose_to_local_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )
        self.setpoint_position_local_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

        self.timer_pub = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.drone_pose_to_wamv = np.identity(4)
        self.drone_pose_to_local = np.identity(4)

    def drone_pose_to_wamv_callback(self, msg):
        self.drone_pose_to_wamv = self.pose_stamped_to_matrix(msg)

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose_to_local = self.pose_stamped_to_matrix(msg)

    def timer_callback(self, event):
        if not np.allclose(self.drone_pose_to_wamv, np.identity(4)):
            if not np.allclose(self.drone_pose_to_local, np.identity(4)):
                wamv_to_drone = np.linalg.inv(self.drone_pose_to_wamv)
                wamv_to_local = np.dot(self.drone_pose_to_local, wamv_to_drone)
                pose_stamped = self.matrix_to_pose_stamped(wamv_to_local, "map")
                pose_stamped.pose.position.z = 1.0
                self.setpoint_position_local_pub.publish(pose_stamped)
                rospy.loginfo_throttle(1.0, "WAMV to local: \n%s", wamv_to_local)
                rospy.loginfo_throttle(1.0, "WAMV to local: \n%s", pose_stamped)
            else:
                rospy.loginfo_throttle(1.0, "Drone pose to local is not available.")
        else:
            rospy.loginfo_throttle(1.0, "Drone pose to WAMV is not available.")

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
