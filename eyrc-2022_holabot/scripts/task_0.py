#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_0.py
# Functions:
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node

# IMPORT MODULES
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# necessary variaables
x=0
y=0
x_init=5.44
y_init=5.44
theta=0
stage=0

# ros message
twist_msg=Twist()
# publisher
pub=rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

# updates pose variables with realtime pose
def pose_callback(data):
    global x, y, theta

    x=data.x
    y=data.y
    theta=data.theta

# main function
def main():
    # node initalise
    rospy.init_node('task_0')
    # subscriber
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    # refering to the global variables
    global x, y, theta, stage

    while not rospy.is_shutdown():
        # draw semicircle
        if(stage==0):
            rotate(0.5, 0.5, theta, 3.14, twist_msg)
        # rotate turtlebot
        if(stage==1):
            rotate(0, 0.5, theta, -1.57, twist_msg)
        # reach origin
        if stage==2:
            move_straight(y, y_init, twist_msg)
        # stop
        if stage==3:
            rospy.signal_shutdown("Task 0 finished")
        
        # publish velocity
        pub.publish(twist_msg)
        
    rospy.spin()      

def move_straight(y, y_init, twist_msg):
    global stage

    # move straight untill target is reached
    if abs(y-y_init)>0.03:
        twist_msg.linear.x=0.5
        twist_msg.angular.z=0
    else:
        stop()
        stage+=1

def rotate(linear_vel, ang_vel, theta, target_orientation, twist_msg):
    global stage

    # rotate untill target orientation is reached
    if abs(theta-target_orientation)>0.03:
        twist_msg.linear.x=linear_vel
        twist_msg.angular.z=ang_vel
    else:
        stop()
        stage+=1

# immediately stops turtlebot
def stop():
    twist_msg.linear.x=0
    twist_msg.angular.z=0

    pub.publish(twist_msg)


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")