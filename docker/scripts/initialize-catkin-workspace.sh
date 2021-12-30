#!/bin/bash

rosdep update
apt-get update
source /opt/ros/eloquent/setup.bash

cd /root/roomba_hack/ros2_ws
rosdep install -y -i --from-paths -r src
colcon build

source /root/roomba_hack/ros2_ws/devel/setup.bash