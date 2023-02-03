#!/usr/bin/env python3

'''
Author: Debrup
Purpose: This server recieves goals via a client and publishes instantaneous velocity of the robot
'''

import rospy
from geometry_msgs.msg import Twist, Pose2D
import math

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
        self.ang_thresh=4*0.01745

        # goals
        self.x_goal=None
        self.y_goal=None
        self.theta_goal=None

        self.rate=rospy.Rate(10)

        # subscriber/publisher
        self.odom_sub=rospy.Subscriber('hb/odom', Pose2D, self.odom_callback)
        self.twist_pub=rospy.Publisher('hb/cmd_vel', Twist, queue_size=10)
        
        # pid params
        self.params_linear={'Kp':0, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':0, 'Ki':0, 'Kd':0}
        self.intg=0
        self.last_error=0

        self.twist_msg=Twist()

        # control loop
        while not rospy.is_shutdown():
            # error calculation
            angle_error=self.theta_goal-self.theta
            error_x=(self.x_goal-self.x)*math.cos(self.theta)+(self.y_goal-self.y)*math.sin(self.theta)
            error_y=-(self.x_-self.x)*math.sin(self.theta)+(self.y_goal-self.y)*math.cos(self.theta)

            # velocity calculation
            v_x, v_y=self.getLinearVel(error_x,  error_y, self.params_linear)
            ang_vel=self.getAngVel(angle_error, self.params_ang)

            # setup the msg for publishing
            self.twist_msg.linear.x=v_x
            self.twist_msg.linear.y=v_y
            self.twist_msg.angular.z=ang_vel

            # publish onto hb/cmd_vel
            self.twist_pub.publish(self.twist_msg)
            self.rate.sleep()

            #stop when reached target pose
            if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh:
                self.stop()
                break
                
    def image_callback(self, msg):
        self.x=msg.x
        self.y=500-msg.y
        self.theta=msg.theta

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

            if ang_vel<0: ang_vel=-1.5
            else: ang_vel=1.5

        else:
            self.stop()

        return ang_vel

    # linear pid function
    def getLinearVel(self, error_x,  error_y, const, x=True):
        v_x=0
        v_y=0
        
        if abs(error_x)>self.linear_thresh or abs(error_y)>self.linear_thresh:
            v_x=self.pid(error_x, const)
            v_y=self.pid(error_y, const)
        else:
            self.stop()

        return v_x, v_y

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