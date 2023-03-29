import rospy
import cv2

def CheckWaypointCorrectness(x_goals, y_goals, theta_goals, mode):
    print("...Waypoint correctness test for {} mode...".format(mode))
    print('Number of trajectories for accross x: {} y: {} theta: {}'.format(len(x_goals[mode]), len(y_goals[mode]), len(theta_goals[mode])))

    if(len(x_goals[mode]) == len(y_goals[mode])) and (len(y_goals[mode]) == len(theta_goals[mode])) and (len(x_goals[mode]) == len(theta_goals[mode])): 
            rospy.loginfo("Number of trajectories consistent accross all degrees!")
    else:
        rospy.loginfo("Check unequal no.of trajectories across each degree of freedom")
        rospy.signal_shutdown("Fix bug")

    for i in range(len(x_goals[mode])):
        if(len(x_goals[mode][i]) != len(y_goals[mode][i])) or (len(y_goals[mode][i]) != len(theta_goals[mode][i])) or (len(x_goals[mode][i]) != len(theta_goals[mode][i])):
            rospy.loginfo("Inconconsistency in no.of waypoints in Trajectory number {}".format(i+1))
            rospy.signal_shutdown("Bug Fix")

    rospy.loginfo("Number of waypoints consistent across all trajectories!")

def DisplayTrajectory(frame, x_goals, y_goals, mode):
    print("Displaying Trajectory for {} mode".format(mode))

    for i in range(len(x_goals[mode])):
        print("Trajectory {}:".format(i+1))
        # some properties of the generated waypoints

        min_px_diff_x=1e8
        max_px_diff_x=-1e8
        min_px_diff_y=1e8
        max_px_diff_y=-1e8
        
        for j in range(len(x_goals[mode][i])):
            if j>0:
                min_px_diff_x=min(min_px_diff_x, abs(x_goals[mode][i][j-1]-x_goals[mode][i][j]))
                max_px_diff_x=max(max_px_diff_x, abs(x_goals[mode][i][j-1]-x_goals[mode][i][j]))

                min_px_diff_y=min(min_px_diff_y, abs(y_goals[mode][i][j-1]-y_goals[mode][i][j]))
                max_px_diff_y=max(max_px_diff_y, abs(y_goals[mode][i][j-1]-y_goals[mode][i][j]))
        
            # blacken out the traversed pixels
            frame[y_goals[mode][i][j]][x_goals[mode][i][j]]=0

            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            rospy.sleep(0.01)

        print(''' Number of waypoints: {}
        Max px gap along x axis: {}
        Min px gap along x axis: {}
        Max px gap along y axis: {}
        Min px gap along y axis: {}'''.format(len(x_goals[mode][i]), max_px_diff_x, min_px_diff_x, max_px_diff_y, min_px_diff_y))

    print("Trajectory visualisation complete!")
