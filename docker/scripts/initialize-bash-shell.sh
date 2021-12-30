#!/bin/bash

alias sh='/bin/bash'
stty -ixon
umask 0002

source /opt/ros/eloquent/setup.bash
source /root/external_ros2_ws/install/local_setup.bash
source /root/roomba_hack/ros2_ws/install/local_setup.bash

export ROS_WORKSPACE=/root/roomba_hack/ros2_ws

function ros_make(){
    dir=$PWD;
    cd $ROS_WORKSPACE;
    if [ $# -gt 0 ]; then
        colcon build --packages-select $1 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release;
    else
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release;
    fi;
    . install/local_setup.bash
    cd $dir
}

cd /root/roomba_hack/