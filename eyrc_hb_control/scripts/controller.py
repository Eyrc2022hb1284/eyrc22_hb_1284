#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup, Sachin
Filename: controller.py
Theme: HoLA Bot
Functions: goal_callback(), odom_callback(), task_stat_callback(), stop() 
Global Variables: None
'''

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from control_utils import *
import ast
from std_msgs.msg import Int32
from eyrc_hb_feed.msg import aruco_data

class goToPose:
    def __init__(self):
        # initalise node
        rospy.init_node('controller')

        # task/pen status storing variables
        self.task_status=1
        self.pen_status=0

        # aruco_data params
        self.x=0
        self.y=0
        self.theta=0

        # tolerance/threshold params
        self.linear_thresh=1
        self.ang_thresh=0.1

        # goal list storing variables
        self.x_goals=None
        self.y_goals=None
        self.theta_goals=None

        # current goal
        self.x_goal=None
        self.y_goal=None
        self.theta_goal=None

        # ros rate
        self.rate=rospy.Rate(75)

        # subscriber/publisher
        self.goal_sub=rospy.Subscriber('/contours', String, self.goal_callback)
        self.odom_sub=rospy.Subscriber('/detected_aruco', aruco_data, self.odom_callback)
        self.task_stat_sub=rospy.Subscriber('/taskStatus', Int32, self.task_stat_callback)

        self.twist_pub=rospy.Publisher('hb/cmd_vel', Twist, queue_size=10)
        self.pen_status_pub=rospy.Publisher('/penStatus', Int32, queue_size=1)
        
        # PID params
        self.params_linear={'Kp':0.0487, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':5, 'Ki':0, 'Kd':0}
        self.intg={'vx':0, 'vy':0, 'w':0}
        self.last_error={'vx':0, 'vy':0, 'w':0}

        # Twist rosmsg
        self.twist_msg=Twist()
        
        # synchronize the script
        while True:
            if self.x_goals is not None and self.y_goals is not None and self.theta_goals is not None:
                break

        # control loop

        # move to homepose(for original homepose)
        self.moveHolaTo(250, 250, 0)
        self.stop()
        rospy.sleep(0.1)

        # iterate through each contour to draw the image/function
        for i in range(len(self.x_goals)):
            # iterate through each point in the contour
            for j in range(len(self.x_goals[i])):

                # move hola to the desired point
                self.moveHolaTo(self.x_goals[i][j], self.y_goals[i][j], self.theta_goals[i][j])

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

        # move back to homepose(for original configuration)
        self.moveHolaTo(250, 250, 0)
        self.stop()
        rospy.sleep(0.1)

    '''
    Function Name: goal_callback
    Input: contour data in the following form-[xListFinal, yListFinal, thetaListFinal]
    Output: No output
    Logic: 
        This function takes in the incoming contour message from the /contours topic, converts it to a list and finally stores the 3 different goal lists
        into 3 different lists (self.x_goals, self.y_goals, self.theta_goals).
    Example call: self.goal_callback(data)
    '''
    def goal_callback(self, data):
        contours = ast.literal_eval(data.data)

        # stores goals
        self.x_goals=contours[0]
        self.y_goals=contours[1]
        self.theta_goals=contours[2]

    '''
    Function Name: goal_callback
    Input: odometry data(x, y, theta)
    Output: No output
    Logic: 
        This function takes in the incoming odometry message from the /detected_aruco topic and stores it 
        into 3 different variables (self.x, self.y, self.theta).
    Example call: self.odom_callback(data)
    '''       
    def odom_callback(self, msg):
        self.x=msg.x 
        self.y=msg.y
        self.theta=msg.theta

    '''
    Function Name: task_stat_callback
    Input: Int32 data
    Output: No output
    Logic: 
        This function takes in the incoming Int32 message(either a 0 or 1 which determines if task starts or ends) 
        from the /taskStatus topic and stores it into 'self.task_status' variable. 
    Example call: self.task_stat_callback(data)
    ''' 
    def task_stat_callback(self, data):
        self.task_status=data.data

    '''
    Function Name: stop
    Input: No input
    Output: No output
    Logic: 
        When called, this function immediately publishes a Twist(linear.x=0, linear.y=0, angular.z=0) 
        onto the hb/cmd_vel rostopic which halts the robot instantaneously.
    Example call: self.stop()
    ''' 
    def stop(self):
        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.twist_msg.angular.z=0
        
        self.twist_pub.publish(self.twist_msg)

    '''
    Function Name: moveHolaTo
    Input: x(x goal), y(y goal), theta(theta goal)
    Output: No output
    Logic: 
        This function moves HoLA bot from one pixel coordinate to another. It encloses the overall bot maneuver logic and thus calls
        the functions that contribute to it(error calculation and getting velocity from them using P controller)
    Example call: self.moveHolaTo(x_goal, y_goal, theta_goal)
    ''' 
    def moveHolaTo(self, x, y, theta):
        self.x_goal=x
        self.y_goal=y
        self.theta_goal=theta

        print("Goal: [{}, {}, {}]".format(self.x_goal, self.y_goal, self.theta_goal))

        # move the bot untill it reaches the goal point
        while True:
            # if aruco marker isn't detected or task status = 1, stop the bot
            if (self.x==-1 and self.y==-1 and self.theta==4) or self.task_status==1 :
                self.stop()
            else:
                # error calculation
                angle_error, error_x, error_y = calculateError([self.x_goal, self.y_goal, self.theta_goal], [self.x, self.y, self.theta])

                # velocity calculation
                v_x, v_y=getLinearVel(error_x,  error_y, self.params_linear, self.linear_thresh, self.intg, self.last_error)
                ang_vel=getAngVel(angle_error, self.params_ang, self.ang_thresh, self.intg, self.last_error)

                # setup the msg for publishing
                self.twist_msg.linear.x=v_x 
                self.twist_msg.linear.y=v_y
                self.twist_msg.angular.z=ang_vel

                # publish onto hb/cmd_vel
                self.twist_pub.publish(self.twist_msg)
                self.rate.sleep()

                #stop when reached target pose
                if abs(angle_error)<=self.ang_thresh and abs(error_x)<=self.linear_thresh and abs(error_y)<=self.linear_thresh:
                    print("reached goal pose: [{}, {}, {}]".format(self.x,  self.y, round(self.theta, 3)))
                    break

if __name__=='__main__':

    gt=goToPose()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)