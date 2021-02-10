# Aquadrone 2020

## First Time Setup and Usage instructions
1. Download and install VirtualBox from this link: https://www.virtualbox.org/wiki/Downloads. The host OS is the operating system that you are running on your computer.
2. Download the Virtual Machine (VM) files from [this link](https://drive.google.com/file/d/1xM4aIALtoE3ixZoi0Vh-BiYUIwNcSH-c/view?usp=sharing) (8.41 GB): 
3. Unzip the downloaded files to your desired location.
4. Open VirtualBox. Click Tools. Click Add. Choose **Aquadrone VM.vbox** in the file chooser dialog.
5. Click the **Aquadrone VM** tab that just appeared. 
6. (optional) Click settings (gear icon). Click system. Change base memory and number of processors to reasonable values for your computer. Half of your system's resources are a good starting point. 
7. Click Start (green arrow icon). The VM's password is **aquadrone**.
8. (recommended) Open a terminal and type ```sudo apt update && sudo apt upgrade -y && git pull && pip3 install -r pip_requirements.txt --upgrade && cd ../../ && catkin clean -y && catkin build```
9. (optional) On the VM home screen click "Search Your Computer", search and select "Screen Display", and change the resolution to whatever best fits your screen. 
10. (optional) Run the following command to see the submarine do a barrel roll: ```roslaunch aquadrone_sim_demos barrel_roll_demo.launch```
11. (optional) If you are having issues pushing to our git repos with your personal account from the VM, use this account: https://github.com/waterloo-aquadrone-vm-account. Message Amaar for the password.

## ROS Crash Course
#### Common Commands
Use tab autocompletion to help find what you’re looking for! Double tap tab to list out all possible autocompletions. Here are some of the most common commands:
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

#### Building the Catkin Workspace
This is required whenever new python packages are created, c++ code is updated, etc.

1. Open a terminal in the aquadrone2020_dev_workspace/catkin_ws/ directory
2. Build the catkin workspace with the following command: ```catkin build```. If stuff is behaving weird and you want to recreate everything to try to avoid potential issues, run the following command instead: ```catkin clean -y && catkin build```.
3. Close all terminals and reopen them so that they initialize properly. Alternatively, you can re-source the workspace in each terminal with the following command: ```source devel/setup.bash```

#### Debugging/Notes
- To run any ROS commands from a terminal (rostopic, roslaunch, rosnode etc), the catkin workspace must be sourced from 
that terminal via the following command: ```source devel/setup.bash```. If the catkin workspace is rebuilt (with the following command: ```catkin build```), this must be redone in each terminal. The VM is configured to do this whenever a new terminal is created (but not when the workspace is rebuilt).
- If you get an error saying that a Python file cannot be found (and you are sure its in the correct location), ensure 
that it is given permission to run as an executable. Open a terminal in the Python file's folder and run the following 
command: ```chmod +x \<file name>.py```. This can also be done outside the VM (i.e. on a windows computer) via git with the command: ```git update-index --chmod=+x \<file name>.py```
- If you are getting weird ROS errors (eg. Gazebo crashing on startup), try updating everything by running the following command in a terminal: ```sudo atp update && sudo apt upgrade -y```

## Aquadrone Specific Specifications

#### Coding Standards
- Topic names should be lowercase, separated by underscores
- Any custom ROS messages should be in the aquadrone_msgs package

#### List of Topics
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

#### List of Nodes
- ekf_state_estimation listens to sensor data and /motor_commands and publishes estimates of the sub's position to /state_estimation
- omniscient_ekf_state_estimation listens to the sub's position from /gazebo/model_states and publishes it directly to /state_estimation
- real_thruster_output listens to /motor_command and sends the PWM signals to control the real thrusters
- thrust_computer listens to various Wrench commands, aggregates them, and publishes the required thruster outputs to /motor_command
- thrust_distributor listens to /motor_command separates the thrusts for each motor and publishes them individually to /aquadrone/thrusters/0/input (same for thrusters 1-7)
- depth_pid listens to /state_estimation and /depth_control/goal_depth and computes the required Wrench based on a PID, and publishes it to /depth_command
- stability listens to /state_estimation and /orientation_target and computes the required Wrench based on a PID, and publishes it to /stability_command
- omniscient_vision_node listens to /gazebo/model_states and computes the exact relative position of objects in the world and publishes them to /Vision_Data

#### Image of a Graph of Nodes and Topics

![Image of a Graph of Nodes and Topics](/path_planning/images/rqt_graph.png)

#### Units
- All units **must** be in metric when appearing on a ROS topic or service
- It is strongly suggested that internal calculations within a ROS node are done in metric
- Values in config files should be in metric where possible, but this is less essential
- Any values/calculations that are not in metric should be clearly noted via comments or variable names

## Development Environment
Although the above process is needed for running the simulations using ROS and Gazebo, the actual code development can be done on whatever platform you want. I would recommend using Pycharm (you can get the professional version for free with a university email address). I would also recommend installing the [Hatchery](https://github.com/duckietown/hatchery) Pycharm plugin.
