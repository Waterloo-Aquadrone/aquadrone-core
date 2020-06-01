# Aquadrone 2020

## First Time Setup and Usage instructions
1. Download VirtualBox from this link: https://www.virtualbox.org/wiki/Downloads
2. Download the Virtual Machine (VM) files from this link (large download): 
https://www.dropbox.com/sh/nf4nli9wn51kszc/AAD_YnMlaH_XO5i9ndWCLnxYa?dl=0&preview=Ubuntu1604ROSKineticSmall.zip
3. Unzip the downloaded files to your desired location.
4. Open VirtualBox. Click Tools. Click Add. Choose Ubuntu1604ROSKineticSmall.vbox in the file chooser dialog.
5. Click the Ubuntu1604ROSKineticSmall tab that just appeared. Click Start.

## Common Commands
- roslaunch \<package name> \<launch file>
- roslaunch \<package name> \<launch file> \<arg name>:=\<arg value>
- rqt_graph
- roscore
- rosnode list
- rostopic list
- rostopic echo \<topic name>
- rostopic pub \<topic name> \<object>
- rosrun \<package name> \<node file>
- catkin build (must be done from the catkin_ws directory)
- source catkin_ws/devel/setup.bash

## Building the Catkin Workspace
This is required whenever new python packages are created, c++ code is updated, etc.

1. Open a terminal in the aquadrone2020dev_workspace/catkin_ws/ directory
2. Build the catkin workspace with the following command: catkin build
3. Source the workspace in all terminals with the following command: source devel/setup.bash

## Debugging/Notes
- To run any ROS commands from a terminal (rostopic, roslaunch, rosnode etc), the catkin workspace must be sourced from 
that terminal via the following command: source devel/setup.bash. The VM is configured to do this whenever a new 
terminal is created. If the catkin workspace is rebuilt (with the following command: catkin build), this must be redone
in each terminal.
- If you get an error saying that a Python file cannot be found (and you are sure its in the correct location), ensure 
that it is given permission to run as an executable. Open a terminal in the Python file's folder and run the following 
command: chmod +x \<file name>.py 

## Coding Standards
- Topic names should be lowercase, separated by underscores
- Any custom ROS messages should be in the aquadrone_msgs package

## List of Topics
- /movement_command send a desired Wrench to the thruster_control system from path_planning
- /stability_command send a desired Wrench to the thruster_control system from orientation PID controller
- /depth_command send a desired Wrench to the thruster_control system from the depth PID controller
- /orientation_target specify the desired orientation of the submarine
- /depth_control/depth_goal specify the desired depth of the submarine
- /state_estimation receive information about the submarine's state
- /Vision_Data receive information about all the objects that are in view of the cameras
- /aquadrone/out/pressure receive information from the pressure (depth) sensor
- /aquadrone/out/imu receive information from the gyro/accelerometer sensor
- /aquadrone/out/front_cam/image_raw receive images from the sub's Gazebo camera
- /motor_command send a list of thrusts for each of the individual thrusters
- /aquadrone/thrusters/0/input send a command to the 0th thruster (same for thrusters 1-7)
- /gazebo/model_states get the state of all objects in the Gazebo simulation

## List of Nodes
- ekf_state_estimation listens to sensor data and /motor_commands and publishes estimates of the sub's position to /state_estimation
- omniscient_ekf_state_estimation listens to the sub's position from /gazebo/model_states and publishes it directly to /state_estimation
- real_thruster_output listens to /motor_command and sends the PWM signals to control the real thrusters
- thrust_computer listens to various Wrench commands, aggregates them, and publishes the required thruster outputs to /motor_command
- thrust_distributor listens to /motor_command separates the thrusts for each motor and publishes them individually to /aquadrone/thrusters/0/input (same for thrusters 1-7)
- depth_pid listens to /state_estimation and /depth_control/goal_depth and computes the required Wrench based on a PID, and publishes it to /depth_command
- stability listens to /state_estimation and /orientation_target and computes the required Wrench based on a PID, and publishes it to /stability_command
- omniscient_vision_node listens to /gazebo/model_states and computes the exact relative position of objects in the world and publishes them to /Vision_Data
