# ROS Cartesian Space Movements

## Description
- This is a ROS project that automatically generate Cartesian space movements of the end-effector of the Panda robot manipulator, The end-effector 'draws' squares of different
sizes on the x-y Cartesian plane, starting from a given robot configuration. This project consists of 2 nodes:
  - 'square_size_generator.py' which generates random pramaters for the size of the square.
  - 'move_panda_square.py' which plans, shows and executs the movement of the panda robot.

## Installation
- Download the 'ros_cartesian_space_movements' package into your catkin workspace 
- If you do not have MoveIt installed, download the following repositories which will run rVis with the panda robot; from the catkin workspace, run the following command;
```
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
```
- And then build your workspace by running the following command: 
```
catkin_make
```

## Running
- from your catkin_workspace activate roscore: 
```
roscore
```

- from your catkin_workspace navigate to the scripts folder: 
```
cd /src/ros_cartesian_space_movements/scripts
```
    
- run the following commands from within the srcipts folder for each node:
```
chmod +x square_size_generator.py
chmod +x move_panda_square.py
```
    
- navigate back to the catkin_ws folder: 
```
cd ~/ catkin_ws
```

- launch a new terminal and run the following node:
```
rosrun ros_cartesian_space_movements square_size_generator.py
```
- in another new terminal, run the following node:
```
rosrun ros_cartesian_space_movements move_panda_square.py
```
- in a final new terminal, run the following command:
```
rosrun rqt_plot rqt_plot
```

## Dependencies
- python 3.8.10
- ros-noetic
- moveit_commander
- moveit_msgs
