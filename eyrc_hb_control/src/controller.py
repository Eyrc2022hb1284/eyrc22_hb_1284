#!/usr/bin/env python3

'''
Author: Debrup
Purpose: This script takes up waypoints from /contours and publishes instantaneous velocity of the robot
'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from control_utils import *
import ast
import cv2
import numpy as np
from std_msgs.msg import Int32
from eyrc_hb_feed.msg import aruco_data

class goToPose:
    def __init__(self):
        # initalise node
        rospy.init_node('controller')

        # task status variable
        self.task_status=1
        self.pen_status=0

        # pose params
        self.x=0
        self.y=0
        self.theta=0

        # threshold params
        self.linear_thresh=1
        self.ang_thresh=0.1

        # goals
        self.x_goals=None
        self.y_goals=None
        self.theta_goals=None

        # current goal
        self.x_goal=None
        self.y_goal=None
        self.theta_goal=None

        self.trajectory=[]

        self.rate=rospy.Rate(75)

        # subscriber/publisher
        self.goal_sub=rospy.Subscriber('/contours', String, self.goal_callback)
        self.odom_sub=rospy.Subscriber('hb/odom', aruco_data, self.odom_callback)
        self.task_stat_sub=rospy.Subscriber('/taskStatus', Int32, self.task_stat_callback)
        self.twist_pub=rospy.Publisher('hb/cmd_vel', Twist, queue_size=10)
        self.pen_status_pub=rospy.Publisher('/penStatus', Int32, queue_size=1)
        
        # pid params
        self.params_linear={'Kp':0.05, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':5, 'Ki':0, 'Kd':0}
        self.intg={'vx':0, 'vy':0, 'w':0}
        self.last_error={'vx':0, 'vy':0, 'w':0}

        self.twist_msg=Twist()
        
        # synchronize the script
        while True:
            if self.x_goals is not None and self.y_goals is not None and self.theta_goals is not None:
                break

        # control loop
        for i in range(len(self.x_goals)):
            for j in range(len(self.x_goals[i])):

                self.x_goal=self.x_goals[i][j]
                self.y_goal=self.y_goals[i][j]
                self.theta_goal=self.theta_goals[i][j]

                print("Goal: [{}, {}, {}]".format(self.x_goal, self.y_goal, self.theta_goal))

                while not rospy.is_shutdown():
                    if self.x==-1 and self.y==-1 and self.theta==4 or self.task_status==1:
                        self.stop()
                    else:
                        # error calculation
                        angle_error=self.theta_goal-self.theta
                        error_x=(self.x_goal-self.x)*math.cos(self.theta)+(self.y-self.y_goal)*math.sin(self.theta)
                        error_y=-(self.x_goal-self.x)*math.sin(self.theta)+(self.y-self.y_goal)*math.cos(self.theta)

                        # velocity calculation
                        v_x, v_y=getLinearVel(error_x,  error_y, self.params_linear, self.linear_thresh, self.intg, self.last_error)
                        ang_vel=getAngVel(angle_error, self.params_ang, self.ang_thresh, self.intg, self.last_error)

                        # setup the msg for publishing
                        self.twist_msg.linear.x=v_x 
                        self.twist_msg.linear.y=v_y
                        self.twist_msg.angular.z=ang_vel

                        # publish onto hb/cmd_vel
                        self.twist_pub.publish(self.twist_msg)
                        # consider odom of the bot as a part of the trajectory only when pen is down
                        if self.pen_status==1: 
                            self.trajectory.append([self.x, self.y])
                        self.rate.sleep()

                        #stop when reached target pose
                        if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh:
                            print("reached goal pose: [{}, {}, {}]".format(self.x,  self.y, round(self.theta, 3)))
                            # if first point of trajectory, pen down
                            if j==0: 
                                self.stop()
                                self.pen_status_pub.publish(1)
                                self.pen_status=1
                                rospy.sleep(0.5)
                            # if last point of trajectory, pen up
                            if j==len(self.x_goals[i])-1: 
                                self.stop()
                                self.pen_status_pub.publish(0)
                                self.pen_status=0
                                rospy.sleep(0.5)

                            break
        
        # trajectory visualisation
        print("Visualising the trajectory...")
        visualiseTrajectory(self.trajectory)
        
        rospy.loginfo("Task completed!")
        rospy.signal_shutdown("Task completed")

    # get the waypoints
    def goal_callback(self, data):
        contours = ast.literal_eval(data.data)

        self.x_goals=contours[0]
        self.y_goals=contours[1]
        self.theta_goals=contours[2]

    # odometry callback             
    def odom_callback(self, msg):
        self.x=msg.x 
        self.y=msg.y
        self.theta=msg.theta

    # updates the task status variable
    def task_stat_callback(self, data):
        self.task_status=data.data

    # bot halt function
    def stop(self):
        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.twist_msg.angular.z=0
        
        self.twist_pub.publish(self.twist_msg)

if __name__=='__main__':

    gt=goToPose()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)