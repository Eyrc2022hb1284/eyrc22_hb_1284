#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup
Filename: contourTest.py
Theme: HoLA Bot
Functions: contour_callback()
Global Variables: None
'''

import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import ast
from test_utils import *

class TestContour:
    def __init__(self):
        rospy.init_node("test_contours")

        # variables storing the goal coordinates
        self.x_goals={'image': None, 'function': None}
        self.y_goals={'image': None, 'function': None}
        self.theta_goals={'image': None, 'function': None}

        # subscriber
        self.sub=rospy.Subscriber('/contours_0', String, self.img_contour_callback)
        self.sub=rospy.Subscriber('/contours_1', String, self.func_contour_callback)

        # ensure that goal points get stored
        rospy.sleep(1)


        # ____________________________________________________Test Script____________________________________________________________________
        # Purpose: -> Check trajectory consistency
        #          -> Check trajectory correctness followed by proper visualisation
        #          -> Calculate resolution of each trajectory
        # ___________________________________________________________________________________________________________________________________

        print("...Waypoint correctness test...")
        if self.x_goals['image'] is not None: CheckWaypointCorrectness(self.x_goals, self.y_goals, self.theta_goals, 'image')
        if self.x_goals['function'] is not None: CheckWaypointCorrectness(self.x_goals, self.y_goals, self.theta_goals, 'function')

        frame=np.ones([500,500,3])

        rospy.loginfo("...Trajectory visualisation test intialised...")
        print("Displaying Trajectory...")

        if self.x_goals['image'] is not None: DisplayTrajectory(frame, self.x_goals, self.y_goals, 'image')
        if self.x_goals['function'] is not None: DisplayTrajectory(frame, self.x_goals, self.y_goals, 'function')

        cv2.destroyAllWindows()

        rospy.loginfo("...Trajectory visualisation test concluded...")
        rospy.signal_shutdown("Test concluded")
    
    '''
    Function Name: contour_callback
    Input: contour data
    Output: None
    Logic: 
        This function takes in the incoming contour data in the form of a string. The data is converted back into a list and then split up to
        extract x, y, theta goals
    Example call: self.contour_callback(data)
    '''
    def img_contour_callback(self, data):
        contours = ast.literal_eval(data.data)

        self.x_goals['image'] = contours[0]
        self.y_goals['image'] = contours[1]
        self.theta_goals['image'] = contours[2]

    def func_contour_callback(self, data):
        contours = ast.literal_eval(data.data)

        self.x_goals['function'] = contours[0]
        self.y_goals['function'] = contours[1]
        self.theta_goals['function'] = contours[2]


if __name__=='__main__':

    tc = TestContour()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)