source /opt/ros/kinetic/setup.bash
source /home/${USER}/catkin_ws/devel/setup.bash

alias killros='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roslaunch & killall -9 roscore & killall -9 rosmaster & killall -9 rviz'