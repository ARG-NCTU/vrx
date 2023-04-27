#! /bin/bash

mkdir /home/argrobotx/robotx-2022/catkin_ws/src/vrx/bags/$(date +%m%d_%H%M)

rosbag record -o /home/argrobotx/robotx-2022/catkin_ws/src/vrx/bags/$(date +%m%d_%H%M)/narrow \
    /gazebo/model_states/throttle \
