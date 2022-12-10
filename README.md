# e-yantra'22 
## Theme-HolA Bot 
Code base for the Holonomic Art bot

## Dependencies and pre-requisites
* ROS Noetic/Ubuntu 20.04

## Setup you environment
make a ROS workspace and go into its src folder
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```
Before continuing source your new setup.*sh file 
```
source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in
```
echo $ROS_PACKAGE_PATH /home/youruser/catkin_ws/src:/opt/ros/noetic/share
```

## Clone the package into the workspace
```
cd src/
git clone https://github.com/sachin7695/eyrc_2022_hb.git
```
restart the terminal and build the package
```
catkin build eyrc-2022_holabot
```

## Using the package
To initialise the robot and the world
```
roslaunch eyrc-2022_holabot gazebo.launch
```
To start the localisation of the HolA bot
```
rosrun eyrc-2022_holabot feedback.py
```
To start the stream of goals and the controller node(This runs the robot)
```
rosrun eyrc-2022_holabot task2exe eyrc-2022_holabot
