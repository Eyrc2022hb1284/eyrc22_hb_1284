#!/usr/bin/env python3

'''
Author: Debrup
Purpose: Test script-visualises the sequence of waypoints generated
'''

import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import ast

class TestContour:
    def __init__(self):
        rospy.init_node("test_contours")

        self.x_goals=None
        self.y_goals=None
        self.theta_goals=None

        self.sub=rospy.Subscriber('/contours', String, self.contour_callback)

        # ensure that goal points get stored
        while not rospy.is_shutdown():
            if self.x_goals is not None and self.y_goals is not None and self.theta_goals is not None:
                break
        
        # print(len(self.x_goals), len(self.y_goals), len(self.theta_goals))

        frame=np.ones([500,500,3])

        # some properties of the generated waypoints
        min_px_diff_x=0
        max_px_diff_x=0
        min_px_diff_y=0
        max_px_diff_y=0

        rospy.loginfo("...Trajectory visualisation test intialised...")
        print("Displaying Trajectory...")

        for i in range(len(self.x_goals)):
            if i>0:
                min_px_diff_x=min(min_px_diff_x, abs(self.x_goals[i-1]-self.x_goals[i]))
                max_px_diff_x=max(max_px_diff_x, abs(self.x_goals[i-1]-self.x_goals[i]))

                min_px_diff_y=min(min_px_diff_y, abs(self.y_goals[i-1]-self.y_goals[i]))
                max_px_diff_y=max(max_px_diff_y, abs(self.y_goals[i-1]-self.y_goals[i]))
           
            # blacken out the traversed pixels
            frame[self.y_goals[i]][self.x_goals[i]]=0

            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            rospy.sleep(5/len(self.x_goals))

        print("Trajectory visuaisation complete!")
        print("Some insights on the trajectory: ")
        print('''
        Max px gap along x axis: {}
        Min px gap along x axis: {}
        Max px gap along y axis: {}
        Min px gap along y axis: {}
        '''.format(max_px_diff_x, min_px_diff_x, max_px_diff_y, min_px_diff_y))

        cv2.destroyAllWindows()

        rospy.loginfo("...Trajectory visualisation test concluded...")
        rospy.signal_shutdown("Test concluded")

    def contour_callback(self, data):
        contours = ast.literal_eval(data.data)

        self.x_goals=contours[0]
        self.y_goals=contours[1]
        self.theta_goals=contours[2]


if __name__=='__main__':

    tc = TestContour()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)