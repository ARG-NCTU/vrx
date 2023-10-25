#!/usr/bin/python3

import rospy
import random
import time
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point, Vector3

class ApplyForceNode:
    def __init__(self):
        rospy.init_node("apply_wamv_force_node")

        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.rate = rospy.Rate(1)

        self.model_name = rospy.get_param("~model_name", "wamv")
        self.link_name = rospy.get_param("~link_name", "wamv/base_link")
        self.dir = [1, -1]
        self.force_x = rospy.get_param("~force_x", 0.0)
        self.force_y = rospy.get_param("~force_y", 0.0)
        self.random_force = rospy.get_param("~randomForce", False)
        self.last_time = time.time()
        if(self.random_force):
            self.force_x = random.uniform(10.0, 13.0)*random.choice(self.dir)
            self.force_y = random.uniform(10.0, 13.0)*random.choice(self.dir)

    def run(self):
        try:

            force = Wrench()
            force.force = Vector3(x=self.force_x, y=self.force_y, z=0.0)
            force.torque = Vector3(x=0.0, y=0.0, z=0.0)

            while not rospy.is_shutdown():
                if self.random_force and (time.time() - self.last_time) > 10.0:
                    self.force_x = random.uniform(10.0, 13.0)*random.choice(self.dir)
                    self.force_y = random.uniform(10.0, 13.0)*random.choice(self.dir)
                    force.force = Vector3(x=self.force_x, y=self.force_y, z=0.0)
                    force.torque = Vector3(x=0.0, y=0.0, z=0.0)
                    self.last_time = time.time()
                body_name_str = self.model_name + "::" + self.link_name
                self.apply_wrench(body_name = body_name_str,
                                  reference_frame = 'world', 
                                  reference_point = Point(x=0.0, y=0.0, z=0.0),
                                  wrench = force,
                                  start_time = rospy.Time(0.0),
                                  duration = rospy.Duration(1.0))
                print(f"force_x: {self.force_x}, force_y: {self.force_y}")
                self.rate.sleep()

        except ValueError:
            pass

if __name__ == "__main__":
    try:
        node = ApplyForceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
