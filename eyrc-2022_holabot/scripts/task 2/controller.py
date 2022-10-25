#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

import numpy as np
import geometry_msgs.msg

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = [249,250,248,251,249]
y_goals = [249,250,251,248,249]
theta_goals = [0,PI/4,-3*PI/4,3*PI/4,0]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	# cleanup()
	sys.exit(0)

# def cleanup():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
  
  
# def task2_goals_Cb(msg):
# 	global x_goals, y_goals, theta_goals
# 	x_goals.clear()
# 	y_goals.clear()
# 	theta_goals.clear()

# 	for waypoint_pose in msg.poses:
# 		x_goals.append(waypoint_pose.position.x)
# 		y_goals.append(waypoint_pose.position.y)

# 		orientation_q = waypoint_pose.orientation
# 		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
# 		theta_goal = euler_from_quaternion (orientation_list)[2]
# 		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg):
	############ ADD YOUR CODE HERE ############
	x=msg.x
	y=msg.y
	theta=msg.theta
	inverse_kinematics(x,y,theta)

	
	
	# orientation_q = msg.pose.pose.orientation
	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################


def inverse_kinematics(x,y,theta):
	############ ADD YOUR CODE HERE ############
	print("hello")
	
	conversion_factor=[[ -0.33 , 0.58 , 0.33],                                                          
 			           [ -0.33 , -0.58 , 0.33],                                                           
 				       [ 0.67 , 0 ,0.33]]
	A=[x-249+1,y-249-1,theta]
	B=np.dot(conversion_factor,A)
	v1,v2,v3=B[0],B[1],B[2]
	print(v1,v2,v3)
	# right_wheel_pub.publish(-76.23)
	# left_wheel_pub.publish(-76.23)
	# front_wheel_pub.publish(154.77)
	time.sleep(1)
	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################
def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['Kp'] * prop + const['Ki'] * self.intg + const['Kd'] * diff
        self.last_error = error
        return balance

    # angular pid function
def getAngVel(self, error, const):
        ang_vel=0

        if abs(error) > self.ang_thresh:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), const)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), const)
            else:
                ang_vel = self.pid(error, const)

            if ang_vel<0: ang_vel=-1
            else: ang_vel=1

        else:
            self.stop(z=True)

        return ang_vel

    # linear pid function
def getLinearVel(self, error_x,  error_y, const, x=True):
        v_x=0
        v_y=0
        
        if abs(error_x)>self.linear_thresh or abs(error_y)>self.linear_thresh:
            v_x=self.pid(error_x, const)
            v_y=self.pid(error_y, const)
        else:
            self.stop(x=True, y=True)

        return v_x, v_y

    # bot halt function
def stop(self, x=False, y=False, z=False):
        if x: self.msg.linear.x = 0
        if y: self.msg.linear.y= 0
        if z: self.msg.angular.z = 0

        self.cmd_pub.publish(self.msg)		

def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)
	pose_msg = Wrench()
	pose_msg.force.x = 0
	pose_msg = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, \
                y = 0, z = 0), \
                torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 100))
	# pose_msg = pose_msg.force.z=10
	# pose_msg.force.x = 0
	# pose_msg.force.y = 0
	# pose_msg.torque.z = 10
	right_wheel_pub.publish(pose_msg)
	# left_wheel_pub.publish()
	# front_wheel_pub.publish()
	rospy.Subscriber('/detected_aruco',Pose2D,aruco_feedback_Cb)
	# rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############
	# pid params
	params_linear={'Kp':1, 'Ki':0, 'Kd':0}
    # params_ang={'Kp':0.5, 'Ki':0, 'Kd':0}
    # intg=0
    # last_error=0

    # ROS msgs
    # msg=Twist()
    # rate=rospy.Rate(10)

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   


	while not rospy.is_shutdown():
		
		# Calculate Error from feedback
		
		# Change the frame by using Rotation Matrix (If you find it required)

		# Calculate the required velocity of bot for the next iteration(s)
		
		# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

		# Apply appropriate force vectors

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

		rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

