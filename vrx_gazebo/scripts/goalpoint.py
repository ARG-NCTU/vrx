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
        # self.pub_1 = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)      
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
        
        self.sub_real_goal = rospy.Subscriber("real_goal", PoseStamped, self.cb_real_goal, queue_size=1)
        self.sub_real_pose = rospy.Subscriber("real_pose", PoseStamped, self.cb_real_pose, queue_size=1)
        self.sub_sim_pose = rospy.Subscriber("sim_pose", PoseStamped, self.cb_sim_pose, queue_size=1)
        
        self.scale_factor = rospy.get_param("~scale_factor", 1.0)
        
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
                
        self.dt_sim_x = None #1510
        self.dt_sim_y = None #60
        
        self.wamv3_x = 1480 #90
        self.wamv3_y = 60 #-30
        
        self.wamv4_x = 1450 #90
        self.wamv4_y = 60 #0

        self.robot_radius = 4
        self.pi2 = math.radians(360)

        self.counter = 0
        self.real_goal, self.real_pose, self.sim_pose = None, None, None
      
    def cb_real_goal(self, msg):
        if self.real_goal is None:
            pass
            
        if msg is not None:
            self.real_goal = msg
            print('real_goal:',msg)
                  
    def cb_real_pose(self, msg):
        self.real_pose = msg
        
    def cb_sim_pose(self, msg):
        self.sim_pose = msg
    
    def goal_to_PoseStamped(self, x, y , orien_w = 0, orien_z = 0):
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = orien_z
        goal.pose.orientation.w = orien_w
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        return goal
    
    
    def cal_real_goal_scale(self):
        matrix_wamv_to_map = self.pose_to_matrix(self.real_pose)
        # wamv_goal_to_map = self.goal_to_PoseStamped(self.real_goal.pose.position.x, self.real_goal.pose.position.x, \
        #                                             self.real_goal.pose.orientation.z, self.real_goal.pose.orientation.w)
        wamv_goal_to_map = self.real_goal
        matrix_wamv_goal_to_map = self.pose_to_matrix(wamv_goal_to_map)
        inv_mat_wamv_goal_to_map = tf_trans.inverse_matrix(matrix_wamv_goal_to_map)
        matrix_goal_to_wamv = np.dot(inv_mat_wamv_goal_to_map, matrix_wamv_to_map)
        
        # scale goal distace for digital twin_ sim if needed , for example, 10 times for jackal to wamv
        matrix_goal_to_wamv[0:2, 3] *= self.scale_factor
    
        matrix_sim_to_map = self.pose_to_matrix(self.sim_pose)
        inv_mat_sim_to_map = tf_trans.inverse_matrix(matrix_sim_to_map)
        inv_matrix_sim_goal_to_map =  np.dot(matrix_goal_to_wamv, inv_mat_sim_to_map)
        matrix_sim_to_map = tf_trans.inverse_matrix(inv_matrix_sim_goal_to_map)
        goal_sim_to_map = self.matrix_to_pose(matrix_sim_to_map, "map")
        # print(wamv_goal_to_map)
        
        self.dt_sim_x = goal_sim_to_map.pose.position.x
        self.dt_sim_y = goal_sim_to_map.pose.position.y
        
        print('-----------------------')
        print('sim_pose: x={:.2f}, y={:.2f}'.format(self.sim_pose.pose.position.x, self.sim_pose.pose.position.y))
        print('sim_goal: x={:.2f}, y={:.2f}'.format(goal_sim_to_map.pose.position.x, goal_sim_to_map.pose.position.y))
        print('dis1: {:.2f}'.format(self.distance(self.sim_pose, goal_sim_to_map)))
        print('real_pose: x={:.2f}, y={:.2f}'.format(self.real_pose.pose.position.x, self.real_pose.pose.position.y))
        print('real_goal: x={:.2f}, y={:.2f}'.format(self.real_goal.pose.position.x, self.real_goal.pose.position.y))
        print('dis2: {:.2f}'.format(self.distance(self.real_pose, self.real_goal)))
        print('-----------------------')

    
        return goal_sim_to_map
    
    def distance(self, pose1, pose2):
        distance = math.sqrt((pose1.pose.position.x - pose2.pose.position.x)**2 + (pose1.pose.position.y - pose2.pose.position.y)**2)
        return distance
    
    def pub_goal(self):       
        if self.counter > 3 and self.counter <15:
            goal_sim_to_map = self.cal_real_goal_scale()
            self.pub_2.publish(goal_sim_to_map)        
            print('wamv2 goal published:', goal_sim_to_map.pose.position.x, goal_sim_to_map.pose.position.y) 

        elif self.counter > 15 and self.counter <30:
            wamv3_goal_to_map = self.goal_to_PoseStamped(self.wamv3_x, self.wamv3_y)
            self.pub_3.publish(wamv3_goal_to_map)
            print('wamv3 goal published:', wamv3_goal_to_map.pose.position.x, wamv3_goal_to_map.pose.position.y)
            
        elif self.counter > 30 and self.counter <45:   
            wamv4_goal_to_map = self.goal_to_PoseStamped(self.wamv4_x, self.wamv4_y)
            self.pub_4.publish(wamv4_goal_to_map)
            print('wamv4 goal published:', wamv4_goal_to_map.pose.position.x, wamv4_goal_to_map.pose.position.y)
            
        else:
            pass
        
    def timer_cb(self,event):
        if self.real_goal is not None and self.real_pose is not None and self.sim_pose is not None:
            self.pub_goal() 
            self.counter += 1
            print("Counter:", self.counter)
        
            if self.real_goal is not None:   
                marker1 = Marker()
                marker1.header.stamp = rospy.Time.now()
                marker1.header.frame_id = 'map'
                marker1.type = marker1.LINE_STRIP
                marker1.action = marker1.ADD
                circumference = []
                for i in range(16+1):
                    p = Point()
                    p.x = self.real_goal.pose.position.x + self.robot_radius* math.cos(i*self.pi2/16)
                    p.y = self.real_goal.pose.position.y + self.robot_radius* math.sin(i*self.pi2/16)
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
            
            if self.dt_sim_x is not None and self.dt_sim_y is not None:   
                marker2 = Marker()
                marker2.header.stamp = rospy.Time.now()
                marker2.header.frame_id = 'map'
                marker2.type = marker2.LINE_STRIP
                marker2.action = marker2.ADD
                circumference = []
                for i in range(16+1):
                    p = Point()
                    p.x = self.dt_sim_x + self.robot_radius* math.cos(i*self.pi2/16)
                    p.y = self.dt_sim_y + self.robot_radius* math.sin(i*self.pi2/16)
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

        else:
            rospy.loginfo('No goal / pose received')
    
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
            if self.counter >= 50:
                rospy.loginfo('Process finished')
                rospy.signal_shutdown('Process finished')
                break
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('Goal_Point',anonymous=False)
    goal_point_node = Goal_Point()
    goal_point_node.run()

    
    


