#! /usr/bin/env python
import math
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from obstacle_detector.msg import Obstacles
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8MultiArray, MultiArrayDimension, MultiArrayLayout
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

class ObstaclesToGrid:
    def __init__(self):
        self.resolution = rospy.get_param("~resolution", 1.0)
        # self.width = rospy.get_param("~width", 100.0)
        self.width = int(150.0 / self.resolution)
        # self.height = rospy.get_param("~height", 100.0)
        self.height = int(150.0 / self.resolution)
        self.origin_x = rospy.get_param("~origin_x", 1400.0)
        self.origin_y = rospy.get_param("~origin_y", -50.0)
        self.origin_z = rospy.get_param("~origin_z", 0.0)
        self.origin_yaw = rospy.get_param("~origin_yaw", 0)
        self.obstacles_sub = rospy.Subscriber("/jackal/raw_obstacles", Obstacles, self.obstacles_callback)
        self.sub_wamv2 = rospy.Subscriber("/gazebo/wamv2/pose", PoseStamped, self.wamv2_pose_callback)
        self.sub_wamv3 = rospy.Subscriber("/gazebo/wamv3/pose", PoseStamped, self.wamv3_pose_callback)
        self.sub_wamv4 = rospy.Subscriber("/gazebo/wamv4/pose", PoseStamped, self.wamv4_pose_callback)
        
        self.reset_sub = rospy.Subscriber("reset_map", Bool, self.reset_callback)
        # self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.grid_pub = rospy.Publisher("grid_map", Int8MultiArray, queue_size=1)
        self.map_pub = rospy.Publisher("obstacle_map", OccupancyGrid, queue_size=1)
        self.pubish_timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback_grid_pub)
        self.obstacles = Obstacles()

        self.grid_map = Int8MultiArray()
        self.grid_map.data = [0] * int(self.width * self.height)
        self.grid_map.layout = MultiArrayLayout()
        self.grid_map.layout.dim = [
            MultiArrayDimension(),
            MultiArrayDimension(),
            MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
            # MultiArrayDimension(),
        ]
        self.grid_map.layout.dim[0].label = "width"
        self.grid_map.layout.dim[0].size = self.width
        self.grid_map.layout.dim[0].stride = self.width * self.height
        self.grid_map.layout.dim[1].label = "height"
        self.grid_map.layout.dim[1].size = self.height
        self.grid_map.layout.dim[1].stride = self.height
        self.grid_map.layout.dim[2].label = "resolution"
        self.grid_map.layout.dim[2].size = self.resolution
        self.grid_map.layout.dim[2].stride = self.resolution
        # self.grid_map.layout.dim[3].label = "origin_x"
        # self.grid_map.layout.dim[3].size = self.origin_x
        # self.grid_map.layout.dim[3].stride = self.origin_x
        # self.grid_map.layout.dim[4].label = "origin_y"
        # self.grid_map.layout.dim[4].size = self.origin_y
        # self.grid_map.layout.dim[4].stride = self.origin_y
        # self.grid_map.layout.dim[5].label = "origin_z"
        # self.grid_map.layout.dim[5].size = self.origin_z
        # self.grid_map.layout.dim[5].stride = self.origin_z
        q = quaternion_from_euler(0, 0, self.origin_yaw)
        # self.grid_map.layout.dim[6].label = "origin_qx"
        # self.grid_map.layout.dim[6].size = q[0]
        # self.grid_map.layout.dim[6].stride = q[0]
        # self.grid_map.layout.dim[7].label = "origin_qy"
        # self.grid_map.layout.dim[7].size = q[1]
        # self.grid_map.layout.dim[7].stride = q[1]
        # self.grid_map.layout.dim[8].label = "origin_qz"
        # self.grid_map.layout.dim[8].size = q[2]
        # self.grid_map.layout.dim[8].stride = q[2]
        # self.grid_map.layout.dim[9].label = "origin_qw"
        # self.grid_map.layout.dim[9].size = q[3]
        # self.grid_map.layout.dim[9].stride = q[3]
        # self.grid_map.layout.data_offset = 0

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = self.width
        self.occupancy_grid.info.height = self.height
        self.occupancy_grid.info.origin.position.x = self.origin_x
        self.occupancy_grid.info.origin.position.y = self.origin_y
        self.occupancy_grid.info.origin.position.z = self.origin_z
        self.occupancy_grid.info.origin.orientation.x = q[0]
        self.occupancy_grid.info.origin.orientation.y = q[1]
        self.occupancy_grid.info.origin.orientation.z = q[2]
        self.occupancy_grid.info.origin.orientation.w = q[3]
        self.occupancy_grid.data = [0] * int(self.width * self.height)

        self.reset_time = None
        # self.joy = Joy()
        self.cnt = 0
    def wamv2_pose_callback(self, msg):
        self.wamv2_pose = msg
    
    def wamv3_pose_callback(self, msg):
        self.wamv3_pose = msg
        
    def wamv4_pose_callback(self, msg):
        self.wamv4_pose = msg

    def transform_obstacle_to_map_frame(self, obstacle_x, obstacle_y):
        theta = self.origin_yaw

        translated_x = obstacle_x - self.origin_x
        translated_y = obstacle_y - self.origin_y

        rotated_x = math.cos(theta) * translated_x - math.sin(theta) * translated_y
        rotated_y = math.sin(theta) * translated_x + math.cos(theta) * translated_y

        return rotated_x, rotated_y

    def scale_obstacle(self, obstacle_x, obstacle_y):
        # scale the distance between the obstacle and the wamv
        factor = 5.0
        wamv_x = self.wamv2_pose.pose.position.x
        wamv_y = self.wamv2_pose.pose.position.y
        vector_to_obstacle = np.array([wamv_x, wamv_y]) - np.array([obstacle_x, obstacle_y])
        scaled_vector =  vector_to_obstacle * factor
        new_obstacle_x = wamv_x - scaled_vector[0]
        new_obstacle_y = wamv_y - scaled_vector[1]
        return new_obstacle_x, new_obstacle_y

    def joy_callback(self, joy):
        if joy.buttons[5] and not self.joy.buttons[5]:
            self.grid_map.data = [0] * int(self.width * self.height)
            self.occupancy_grid.data = [0] * int(self.width * self.height)
            self.reset_time = rospy.Time.now()
            rospy.logwarn("Resetting map")
        self.joy = joy

    def reset_callback(self, msg):
        self.grid_map.data = [0] * int(self.width * self.height)
        self.occupancy_grid.data = [0] * int(self.width * self.height)
        self.reset_time = rospy.Time.now()
        rospy.logwarn("Resetting map")

    def obstacles_callback(self, msg):
        if self.reset_time is not None and rospy.Time.now() - self.reset_time < rospy.Duration(1.0):
            return
        self.obstacles = msg
                
        for center in msg.circles:
            scale_x, scale_y = self.scale_obstacle(center.center.x, center.center.y)
            map_x, map_y = self.transform_obstacle_to_map_frame(scale_x, scale_y)
            
            # map_x, map_y = self.transform_obstacle_to_map_frame(center.center.x, center.center.y)
            map_x = map_x / self.resolution
            map_y = map_y / self.resolution
            scan_size = 20
            for dx in np.arange(-scan_size + 1, scan_size, 1):
                for dy in np.arange(-scan_size, scan_size, 1):
                    new_x = map_x + dx
                    new_y = map_y + dy
                    if 0 <= new_x < self.width and 0 <= new_y < self.height:
                        new_ind = int(int(new_x) + int(new_y) * self.width)
                        # new_ind = int(int(new_y) + int(new_x) * self.width)
                        if self.grid_map.data[new_ind]:
                            self.grid_map.data[new_ind] = 0.0
                            self.occupancy_grid.data[new_ind] = 0
        for center in msg.circles:
            scale_x, scale_y = self.scale_obstacle(center.center.x, center.center.y)
            if 27.0<= scale_y <= 33.0:
                return
            elif self.distance(self.wamv3_pose, scale_x, scale_y) <= 5.0:
                return
            elif self.distance(self.wamv4_pose, scale_x, scale_y) <= 5.0:
                return
            map_x, map_y = self.transform_obstacle_to_map_frame(scale_x, scale_y)

            # map_x, map_y = self.transform_obstacle_to_map_frame(center.center.x, center.center.y)
            map_x = map_x / self.resolution
            map_y = map_y / self.resolution

            if 0 <= map_x < self.width and 0 <= map_y < self.height:
                index = int(map_x) + int(map_y) * self.width
                # index = int(int(new_y) + int(new_x) * self.width)
                self.grid_map.data[int(index)] = 1.0
                self.occupancy_grid.data[int(index)] = 100
            else:
                rospy.logwarn("Obstacle out of map bounds: ({}, {})".format(map_x, map_y))
                
    def distance(self,pose , x2, y2):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def timer_callback_grid_pub(self, event):
        self.cnt += 1
        if self.cnt % 2 == 0:
            self.grid_pub.publish(self.grid_map)
            self.cnt = 0
            
        self.map_pub.publish(self.occupancy_grid)
        obs_len = len(self.obstacles.circles)
        grid_len = len([x for x in self.grid_map.data if x == 1])
        rospy.loginfo_throttle(1, "Obstacles: {}, Grid: {}".format(obs_len, grid_len))


if __name__ == "__main__":
    rospy.init_node("obstacles_map_scale_dis_btw_USVandObstacle")
    ObstaclesToGrid()
    rospy.spin()