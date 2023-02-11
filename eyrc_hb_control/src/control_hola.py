#!/usr/bin/env python3

'''
Author: Debrup
Purpose: This server recieves goals via a client and publishes instantaneous velocity of the robot
'''

import rospy
from geometry_msgs.msg import Twist, Pose2D
import math
from control_utils import *

class goToPose:
    def __init__(self):
        # initalise node
        rospy.init_node('error_pub')

        # pose params
        self.x=0
        self.y=0
        self.theta=0

        # threshold params
        self.linear_thresh=1
        self.ang_thresh=0.1

        # goals
        self.x_goal=250
        self.y_goal=250
        self.theta_goal=0

        self.rate=rospy.Rate(75)

        # subscriber/publisher
        self.odom_sub=rospy.Subscriber('hb/odom', Pose2D, self.odom_callback)
        self.twist_pub=rospy.Publisher('hb/cmd_vel', Twist, queue_size=10)
        
        # pid params
        self.params_linear={'Kp':0.025, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':1, 'Ki':0, 'Kd':0}
        self.intg={'vx':0, 'vy':0, 'w':0}
        self.last_error={'vx':0, 'vy':0, 'w':0}

        self.twist_msg=Twist()

        # control loop
        while not rospy.is_shutdown():
            # error calculation
            # print("theta: {}".format(self.theta))

            angle_error=self.theta_goal-self.theta
            error_x=(self.x_goal-self.x)*math.cos(self.theta)+(self.y_goal-self.y)*math.sin(self.theta)
            error_y=-(self.x_goal-self.x)*math.sin(self.theta)+(self.y_goal-self.y)*math.cos(self.theta)

            # velocity calculation
            v_x, v_y=getLinearVel(error_x,  error_y, self.params_linear, self.linear_thresh, self.intg, self.last_error)
            ang_vel=getAngVel(angle_error, self.params_ang, self.ang_thresh, self.intg, self.last_error)

            print("vx: {} vy: {} Ang vel: {}".format(v_x, v_y, ang_vel))

            # setup the msg for publishing
            self.twist_msg.linear.x=v_x
            self.twist_msg.linear.y=v_y
            self.twist_msg.angular.z=ang_vel

            # publish onto hb/cmd_vel
            self.twist_pub.publish(self.twist_msg)
            self.rate.sleep()

            #stop when reached target pose
            # if abs(angle_error)<=self.ang_thresh:
            if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh:
                print("Halting")
                self.stop()
                # break
                
    def odom_callback(self, msg):
        self.x=msg.x
        self.y=500-msg.y
        self.theta=msg.theta

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