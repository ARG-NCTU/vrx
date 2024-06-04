#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
import argparse
import sys

class SetModelPose:
    def __init__(self, model_name, position, orientation):
        self.model_name = model_name
        self.position = position
        self.orientation = orientation
        self.service_name = 'reset_model_{}'.format(model_name)
        rospy.init_node('reset_model_service')
        s = rospy.Service(self.service_name, Empty, self.reset_model)
        rospy.loginfo('Ready to reset {} pose.'.format(self.model_name))
        rospy.spin()

    def reset_model(self, req):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = float(self.position[0])
            model_state.pose.position.y = float(self.position[1])
            model_state.pose.position.z = float(self.position[2])
            model_state.pose.orientation.x = float(self.orientation[0])
            model_state.pose.orientation.y = float(self.orientation[1])
            model_state.pose.orientation.z = float(self.orientation[2])
            model_state.pose.orientation.w = float(self.orientation[3])
            req = SetModelStateRequest()
            req.model_state = model_state
            set_model_state(req)
            return EmptyResponse()
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))
            return EmptyResponse()

if __name__ == "__main__":
    # Filter out ROS-specific arguments
    args = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description='Set the pose of a Gazebo model.')
    parser.add_argument('--model_name', type=str, default='red_pole', help='Name of the model to reset')
    parser.add_argument('-x', type=float, required=True, help='Position x')
    parser.add_argument('-y', type=float, required=True, help='Position y')
    parser.add_argument('-z', type=float, required=True, help='Position z')
    parser.add_argument('-rx', type=float, required=True, help='Orientation roll (rx)')
    parser.add_argument('-ry', type=float, required=True, help='Orientation pitch (ry)')
    parser.add_argument('-rz', type=float, required=True, help='Orientation yaw (rz)')
    parser.add_argument('-rw', type=float, default=1.0, help='Orientation w (rw)')

    parsed_args = parser.parse_args(args[1:])

    position = [parsed_args.x, parsed_args.y, parsed_args.z]
    orientation = [parsed_args.rx, parsed_args.ry, parsed_args.rz, parsed_args.rw]

    set_model_pose = SetModelPose(parsed_args.model_name, position, orientation)
