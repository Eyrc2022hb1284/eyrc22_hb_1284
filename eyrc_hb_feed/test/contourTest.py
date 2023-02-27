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
        
        print("...Waypoint correctness test...")
        print('Number of trajectories accross x: {} y: {} theta: {}'.format(len(self.x_goals), len(self.y_goals), len(self.theta_goals)))

        if(len(self.x_goals) == len(self.y_goals)) and (len(self.y_goals) == len(self.theta_goals)) and (len(self.x_goals) == len(self.theta_goals)): 
            rospy.loginfo("Number of trajectories consistent accross all degrees!")
        else:
            rospy.loginfo("Check unequal no.of trajectories across each degree of freedom")
            rospy.signal_shutdown("Fix bug")

        for i in range(len(self.x_goals)):
            if(len(self.x_goals[i]) != len(self.y_goals[i])) or (len(self.y_goals[i]) != len(self.theta_goals[i])) or (len(self.x_goals[i]) != len(self.theta_goals[i])):
                rospy.loginfo("Inconconsistency in no.of waypoints in Trajectory number {}".format(i+1))
                rospy.signal_shutdown("Bug Fix")
        rospy.loginfo("Number of waypoints consistent across all trajectories!")
        
        frame=np.ones([500,500,3])

        rospy.loginfo("...Trajectory visualisation test intialised...")
        print("Displaying Trajectory...")

        for i in range(len(self.x_goals)):
            print("Trajectory {}:".format(i+1))
            # some properties of the generated waypoints

            min_px_diff_x=0
            max_px_diff_x=0
            min_px_diff_y=0
            max_px_diff_y=0
            for j in range(len(self.x_goals[i])):
                if j>0:
                    min_px_diff_x=min(min_px_diff_x, abs(self.x_goals[i][j-1]-self.x_goals[i][j]))
                    max_px_diff_x=max(max_px_diff_x, abs(self.x_goals[i][j-1]-self.x_goals[i][j]))

                    min_px_diff_y=min(min_px_diff_y, abs(self.y_goals[i][j-1]-self.y_goals[i][j]))
                    max_px_diff_y=max(max_px_diff_y, abs(self.y_goals[i][j-1]-self.y_goals[i][j]))
            
                # blacken out the traversed pixels
                frame[self.y_goals[i][j]][self.x_goals[i][j]]=0

                cv2.imshow('frame', frame)
                cv2.waitKey(1)
                rospy.sleep(0.001)

            print(''' Number of waypoints: {}
            Max px gap along x axis: {}
            Min px gap along x axis: {}
            Max px gap along y axis: {}
            Min px gap along y axis: {}'''.format(len(self.x_goals[i]), max_px_diff_x, min_px_diff_x, max_px_diff_y, min_px_diff_y))

        print("Trajectory visualisation complete!")
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