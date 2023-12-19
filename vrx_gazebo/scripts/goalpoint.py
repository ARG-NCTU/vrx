#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
class goal_point():
    def __init__(self):

        # self.pub_1 = rospy.Publisher("/wamv1/move_base_simple/goal", PoseStamped, queue_size=1)   
        self.pub_1 = rospy.Publisher("/wamv/move_base_simple/goal", PoseStamped, queue_size=1)     
        self.pub_2 = rospy.Publisher("/wamv2/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_3 = rospy.Publisher("/wamv3/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_4 = rospy.Publisher("/wamv4/move_base_simple/goal", PoseStamped, queue_size=1)

        # self.pub_position_circle = rospy.Publisher("/visualization_circle", Marker, queue_size=1)
        self.pub_position_circle1 = rospy.Publisher("/visualization_circle1", Marker, queue_size=1)
        self.pub_position_circle2 = rospy.Publisher("/visualization_circle2", Marker, queue_size=1)
        self.pub_position_circle3 = rospy.Publisher("/visualization_circle3", Marker, queue_size=1)
        self.pub_position_circle4 = rospy.Publisher("/visualization_circle4", Marker, queue_size=1)
        # self.pub_position_circle5 = rospy.Publisher("/visualization_circle5", Marker, queue_size=1)
        self.pub_map1 = rospy.Publisher("/visualization_map1", Marker, queue_size=1)
        self.pub_map2 = rospy.Publisher("/visualization_map2", Marker, queue_size=1)
        self.pub_map3 = rospy.Publisher("/visualization_map3", Marker, queue_size=1)
        self.pub_map4 = rospy.Publisher("/visualization_map4", Marker, queue_size=1)
        
        
        self.pub_state_to_mapgrid = rospy.Publisher("/reset_map", Bool, queue_size=1)
        self.joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1), self.cb_publish)
        
        
        self.wamv2_x = 90
        self.wamv2_y = 0
        
        self.wamv3_x = 90
        self.wamv3_y = -30
        
        self.wamv4_x = 90
        self.wamv4_y = 30

        self.robot_radius = 3
        self.pi2 = math.radians(360)
        
        self.sub_wamv = rospy.Subscriber("/gazebo/wamv/pose", PoseStamped, self.cb_wamv, queue_size=1)
        self.wamv_x, self.wamv_y, self.wamv_z, self.wamv_qx, self.wamv_qy, self.wamv_qz, self.wamv_qw = 0,0,0,0,0,0,0
        
        self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.counter = 0
        print('reset USV pose')


    def cb_joy(self, msg):
        if msg.buttons[4] == 1:
            self.set_wamv_pose(model_name='wamv2', x=10 , y=0, z=self.wamv_z, qx=self.wamv_qx, qy=self.wamv_qy, qz=self.wamv_qz, qw=self.wamv_qw)
            self.set_wamv_pose(model_name='wamv3', x=10 , y=50, z= -0.090229, qx=0, qy=0, qz=0, qw=0)
            self.set_wamv_pose(model_name='wamv4', x=10 , y=-50, z= -0.090229, qx=0, qy=0, qz=0, qw=0)
            self.counter = 0
        else:
            pass
        
    def cb_wamv(self, msg):
        self.wamv_x = msg.pose.position.x
        self.wamv_y = msg.pose.position.y
        self.wamv_z = msg.pose.position.z
        self.wamv_qx = msg.pose.orientation.x
        self.wamv_qy = msg.pose.orientation.y
        self.wamv_qz = msg.pose.orientation.z
        self.wamv_qw = msg.pose.orientation.w
    
    def pub_goal(self):
        print('counter:', self.counter)
        
        if self.counter == 2 :
            self.pub_state_to_mapgrid.publish(True)
        
        if self.counter > 5 and self.counter <19:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.wamv2_x
            pose.pose.position.y = self.wamv2_y
            self.pub_1.publish(pose)
            self.pub_2.publish(pose)
            
            print('wamv2 goal published:', self.wamv2_x, self.wamv2_y)
            
        elif self.counter > 20 and self.counter <34:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.wamv3_x
            pose.pose.position.y = self.wamv3_y
            self.pub_3.publish(pose)
            print('wamv3 goal published:', self.wamv3_x, self.wamv3_y)
            
        elif self.counter > 35 and self.counter <49:    
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.wamv4_x
            pose.pose.position.y = self.wamv4_y
            self.pub_4.publish(pose)
            print('wamv4 goal published:', self.wamv4_x, self.wamv4_y)
            
        # elif self.counter >= 55:
        #     rospy.signal_shutdown("Counter reached 50, shutting down.")
            # return
        else:
            pass
            
    def cb_publish(self,event):

        self.pub_goal()
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
        self.pub_position_circle1.publish(marker2)
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
        self.pub_map1.publish(marker)

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

    def set_wamv_pose(self, model_name='wamv2',x=0 , y=0, z=0, qx=0, qy=0, qz=0, qw=0):
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.reference_frame = "world"
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        model_state.pose.orientation.x = qx
        model_state.pose.orientation.y = qy
        model_state.pose.orientation.z = qz
        model_state.pose.orientation.w = qw
        
        self.set_model_state.publish(model_state)
        

if __name__ == '__main__':
    rospy.init_node('goal_point_node',anonymous=False)
    goal_point_node = goal_point()
    #rospy.on_shutdown(goal_point_node.on_shutdown)
    rospy.spin()
    
    
    


