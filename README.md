# E-yantra'22 Rank-1 
## Theme-HolA Bot 
#Youtube Link
https://www.youtube.com/watch?v=rp9mCYZBdvU&t=167s
<br/>
Code base for the Holonomic Art bot

## Dependencies and pre-requisites
* ROS Noetic/Ubuntu 20.04

## Setup you environment
make a ROS workspace and go into its src folder
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```
Before continuing source your new setup.*sh file 
```sh
source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in
```sh
echo $ROS_PACKAGE_PATH /home/youruser/catkin_ws/src:/opt/ros/noetic/share
```

## Clone the package into the workspace
```sh
cd src/
git clone https://github.com/sachin7695/eyrc_2022_hb.git
```
restart the terminal and build the package
```sh
catkin build eyrc-2022_holabot
```

## Using the package
To initialise the robot and the world
```sh
roslaunch eyrc-2022_holabot gazebo.launch
```
To start the localisation of the HolA bot
```sh
rosrun eyrc-2022_holabot feedback.py
```
To start the stream of goals and the controller node(This runs the robot)
```sh
rosrun eyrc-2022_holabot task2exe eyrc-2022_holabot
```
To run the hola bot for functional plotting 
```sh 
roslaunch eyrc_hb_feed getOdom.launch 

rosrun eyrc_hb_feed getContours.py -m 0/1 -p {no of points}

rosrun eyrc_hb_communication hola_transmitter.py 192.168.147.82 -p 44444  

rosrun eyrc_hb_control controller.py
