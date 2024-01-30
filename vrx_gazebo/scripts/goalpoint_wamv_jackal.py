#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf.transformations as tf_trans
import numpy as np

class Goal_Point():
    def __init__(self):
        self.pub_1 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)      
        self.pub_2 = rospy.Publisher("/wamv2/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_3 = rospy.Publisher("/wamv3/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_4 = rospy.Publisher("/wamv4/move_base_simple/goal", PoseStamped, queue_size=1)

        self.pub_position_circle1 = rospy.Publisher("/visualization_circle1", Marker, queue_size=1)
        self.pub_position_circle2 = rospy.Publisher("/visualization_circle2", Marker, queue_size=1)
        self.pub_position_circle3 = rospy.Publisher("/visualization_circle3", Marker, queue_size=1)
        self.pub_position_circle4 = rospy.Publisher("/visualization_circle4", Marker, queue_size=1)

        self.pub_map1 = rospy.Publisher("/visualization_map1", Marker, queue_size=1)
        self.pub_map2 = rospy.Publisher("/visualization_map2", Marker, queue_size=1)
        self.pub_map3 = rospy.Publisher("/visualization_map3", Marker, queue_size=1)
        self.pub_map4 = rospy.Publisher("/visualization_map4", Marker, queue_size=1)
        
        self.sub_real_pose = rospy.Subscriber("/jackal/slam_pose", PoseStamped, self.cb_real_pose, queue_size=1)
        # self.sub_real_pose = rospy.Subscriber("/gazebo/wamv3/pose", PoseStamped, self.cb_real_pose, queue_size=1)
        self.sub_sim_pose = rospy.Subscriber("/gazebo/wamv2/pose", PoseStamped, self.cb_sim_pose, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        
        self.jackal_x = None
        self.jackal_y = None
        
        self.wamv2_x = 1510
        self.wamv2_y = 60
        
        self.wamv3_x = 1480
        self.wamv3_y = 60
        
        self.wamv4_x = 1450
        self.wamv4_y = 60

        self.robot_radius = 4
        self.pi2 = math.radians(360)

        self.counter = 0

    def cb_real_pose(self, msg):
        self.real_pose = msg
        
    def cb_sim_pose(self, msg):
        self.sim_pose = msg
    
    def goal_to_PoseStamped(self, x, y):
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = 0.707
        goal.pose.orientation.w = 0.707
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        return goal
    
    def cal_real_goal_scale(self):
        matrix_wamv2_to_map = self.pose_to_matrix(self.sim_pose)
        wamv2_goal_to_map = self.goal_to_PoseStamped(self.wamv2_x, self.wamv2_y)
        matrix_wamv2_goal_to_map = self.pose_to_matrix(wamv2_goal_to_map)
        inv_mat_wamv2_goal_to_map = tf_trans.inverse_matrix(matrix_wamv2_goal_to_map)
        matrix_goal_to_wamv2 = np.dot(inv_mat_wamv2_goal_to_map, matrix_wamv2_to_map)
        # print(matrix_goal_to_wamv2)
        # print(matrix_goal_to_wamv2[0:2])
        # print(matrix_goal_to_wamv2[0:2].shape)

        # print(matrix_goal_to_wamv2.shape)
        # print(matrix_goal_to_wamv2)
        matrix_goal_to_wamv2[0:2, 3] *= 0.1
        # matrix_goal_to_wamv2[0][3] *= 0.1
        # matrix_goal_to_wamv2[1][3] *= 0.1
        
        matrix_jackal_to_map = self.pose_to_matrix(self.real_pose)
        inv_mat_jackal_to_map = tf_trans.inverse_matrix(matrix_jackal_to_map)
        inv_matrix_jackal_goal_to_map =  np.dot(matrix_goal_to_wamv2, inv_mat_jackal_to_map)
        matrix_jackal_to_map = tf_trans.inverse_matrix(inv_matrix_jackal_goal_to_map)
        goal_jackal_to_map = self.matrix_to_pose(matrix_jackal_to_map, "map")
        
        print('-----------------------')
        print('sim_pose:', self.sim_pose.pose.position.x, self.sim_pose.pose.position.y)
        print('sim_goal:', wamv2_goal_to_map.pose.position.x, wamv2_goal_to_map.pose.position.y)
        print('dis1:', self.distance(self.sim_pose, wamv2_goal_to_map))
        print('real_pose:', self.real_pose.pose.position.x, self.real_pose.pose.position.y)
        print('real_goal:', goal_jackal_to_map.pose.position.x, goal_jackal_to_map.pose.position.y)
        print('dis2:', self.distance(self.real_pose, goal_jackal_to_map))
        print('-----------------------')
    
        return goal_jackal_to_map
    def distance(self, pose1, pose2):
        distance = math.sqrt((pose1.pose.position.x - pose2.pose.position.x)**2 + (pose1.pose.position.y - pose2.pose.position.y)**2)
        return distance
    
    def pub_goal(self):
        print('counter:', self.counter)       
        goal_jackal_to_map = self.cal_real_goal_scale()
        
        if self.counter > 5 and self.counter <19:
            wamv2_goal_to_map = self.goal_to_PoseStamped(self.wamv2_x, self.wamv2_y)
            self.pub_2.publish(wamv2_goal_to_map)        
            print('wamv2 goal published:', wamv2_goal_to_map.pose.position.x, wamv2_goal_to_map.pose.position.y)      
            self.pub_1.publish(goal_jackal_to_map)
            print('jackal goal published:', goal_jackal_to_map.pose.position.x, goal_jackal_to_map.pose.position.y) 
            self.jackal_x = goal_jackal_to_map.pose.position.x
            self.jackal_y = goal_jackal_to_map.pose.position.y
            
        elif self.counter > 20 and self.counter <34:
            wamv3_goal_to_map = self.goal_to_PoseStamped(self.wamv3_x, self.wamv3_y)
            self.pub_3.publish(wamv3_goal_to_map)
            print('wamv3 goal published:', wamv3_goal_to_map.pose.position.x, wamv3_goal_to_map.pose.position.y)
            
        elif self.counter > 35 and self.counter <49:   
            wamv4_goal_to_map = self.goal_to_PoseStamped(self.wamv4_x, self.wamv4_y)
            self.pub_4.publish(wamv4_goal_to_map)
            print('wamv4 goal published:', wamv4_goal_to_map.pose.position.x, wamv4_goal_to_map.pose.position.y)
            
        else:
            pass
        
    def timer_cb(self,event):
        self.pub_goal() 
        
        if self.jackal_x is not None and self.jackal_y is not None:
            marker1 = Marker()
            marker1.header.stamp = rospy.Time.now()
            marker1.header.frame_id = 'map'
            marker1.type = marker1.LINE_STRIP
            marker1.action = marker1.ADD
            circumference = []
            for i in range(16+1):
                p = Point()
                p.x = self.jackal_x + 1.5* math.cos(i*self.pi2/16)
                p.y = self.jackal_y + 1.5* math.sin(i*self.pi2/16)
                p.z = 0.1
                circumference.append(p)

            marker1.pose.orientation.w = 1
            marker1.points = circumference
            marker1.scale.x = 0.1
            marker1.color.a = 1.0
            marker1.color.r = 0
            marker1.color.g = 1
            marker1.color.b = 0
            self.pub_position_circle1.publish(marker1)
            
        marker2 = Marker()
        marker2.header.stamp = rospy.Time.now()
        marker2.header.frame_id = 'map'
        marker2.type = marker2.LINE_STRIP
        marker2.action = marker2.ADD
        circumference = []
        for i in range(16+1):
            p = Point()
            p.x = self.wamv2_x + self.robot_radius* math.cos(i*self.pi2/16)
            p.y = self.wamv2_y + self.robot_radius* math.sin(i*self.pi2/16)
            p.z = 0.1
            circumference.append(p)

        marker2.pose.orientation.w = 1
        marker2.points = circumference
        marker2.scale.x = 0.1
        marker2.color.a = 1.0
        marker2.color.r = 0
        marker2.color.g = 1
        marker2.color.b = 0
        #wamv & wamv2
        self.pub_position_circle2.publish(marker2)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []
        for i in range(16+1):
            p = Point()
            p.x = self.wamv3_x + self.robot_radius* math.cos(i*self.pi2/16)
            p.y = self.wamv3_y + self.robot_radius* math.sin(i*self.pi2/16)
            p.z = 0.1
            circumference.append(p)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        self.pub_position_circle3.publish(marker)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []
        for i in range(16+1):
            p = Point()
            p.x = self.wamv4_x + self.robot_radius* math.cos(i*self.pi2/16)
            p.y = self.wamv4_y + self.robot_radius* math.sin(i*self.pi2/16)
            p.z = 0.1
            circumference.append(p)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        self.pub_position_circle4.publish(marker)


        #map####################################
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []
        p = Point()
        p.x = 30
        p.y = -115
        p.z = 0.1
        p1 = Point()
        p1.x = 30
        p1.y = -260
        p1.z = 0.1
        p2 = Point()
        p2.x = -150
        p2.y = -260
        p2.z = 0.1
        p3 = Point()
        p3.x = -150
        p3.y = -115
        p3.z = 0.1

        circumference.append(p)
        circumference.append(p1)
        circumference.append(p2)
        circumference.append(p3)
        circumference.append(p)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0.5
        marker.color.b = 0.5
        # self.pub_map1.publish(marker)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []

        p5 = Point()
        p5.x = -60
        p5.y = -260
        p5.z = 0.1
        p6 = Point()
        p6.x = -60
        p6.y = -195
        p6.z = 0.1

        circumference.append(p5)
        circumference.append(p6)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0.5
        marker.color.b = 0.5
        self.pub_map2.publish(marker)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        circumference = []

        p4 = Point()
        p4.x = -60
        p4.y = -115
        p4.z = 0.1
        p7 = Point()
        p7.x = -60
        p7.y = -185
        p7.z = 0.1

        circumference.append(p4)
        circumference.append(p7)

        marker.pose.orientation.w = 1
        marker.points = circumference
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0.5
        marker.color.b = 0.5
        self.pub_map3.publish(marker)

        self.counter += 1
        # print(self.counter)
    
    def pose_to_matrix(self, pose):
        if isinstance(pose, PoseStamped):
            translation = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            rotation = [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]
            return tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(translation), tf_trans.quaternion_matrix(rotation)
            )

    def matrix_to_pose(self, matrix, frame_id="map"):
        ret_pose_stamped = PoseStamped()
        trans = tf_trans.translation_from_matrix(matrix)
        rot = tf_trans.quaternion_from_matrix(matrix)
        ret_pose_stamped.header.frame_id = frame_id
        ret_pose_stamped.header.stamp = rospy.Time.now()

        ret_pose_stamped.pose.position.x = trans[0]
        ret_pose_stamped.pose.position.y = trans[1]
        ret_pose_stamped.pose.position.z = trans[2]
        ret_pose_stamped.pose.orientation.x = rot[0]
        ret_pose_stamped.pose.orientation.y = rot[1]
        ret_pose_stamped.pose.orientation.z = rot[2]
        ret_pose_stamped.pose.orientation.w = rot[3]
        return ret_pose_stamped
    
    def run(self): 
        while not rospy.is_shutdown():
            if self.counter >= 55:
                rospy.loginfo('Process finished')
                rospy.signal_shutdown('Process finished')
                break
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('Goal_Point',anonymous=False)
    goal_point_node = Goal_Point()
    goal_point_node.run()

    
    


