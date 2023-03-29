'''
Team Id: HB1284
Author List: Debrup, Sachin
Filename: control_utils.py
Theme: HoLA Bot
Functions: calculateError(), getAngVel(), getLinearVel(), pid(), visualiseTrajectory()
Global Variables: None
'''

import numpy as np
import rospy
import cv2
import math

'''
Function Name: calculateError
Input: goal(dictionary storing x, y, theta goals), pose(dictionary storing the current pose/odometry)
Output: angular and linear error(along x and y axes)
Logic: 
    This function computes the instantaneous angular and linear error wrt the current goal using translational and rotational transformations.
Example call: angle_error, error_x, error_y = calculateError(goal, pose)
'''
def calculateError(goal, pose):
    # unpacking data into variables
    x_goal, y_goal, theta_goal=goal['x'], goal['y'], goal['theta']
    x, y, theta=pose['x'], pose['y'], pose['theta']

    # error calculation
    angle_error=theta_goal-theta
    error_x=(x_goal-x)*math.cos(theta)+(y-y_goal)*math.sin(theta)
    error_y=-(x_goal-x)*math.sin(theta)+(y-y_goal)*math.cos(theta)

    return angle_error, error_x, error_y

'''
Function Name: pid
Input: error, PID parameters, intg, last_error
Output: balance
Logic: 
    This function takes in the error and returns a balanced output according to the paramters given to it. It has 3 parameters:
    Kp(proportionality constant), Kd(differentiability constant) and Ki(Integral constant)
Example call: balance = pid(error, const, intg, last_error)
'''
def pid(error, const, intg, last_error):
        prop = error
        intg = error + intg
        diff = error - last_error
        # compute balance
        balance = const['Kp'] * prop + const['Ki'] * intg + const['Kd'] * diff
        # update the last error
        last_error = error

        return balance

'''
Function Name: getAngVel
Input: angular_error, PID params, threshold dictionary, intg and last_error params
Output: angular velocity
Logic: 
    This function computes the intantaneous angular velocity of the robot by passing the instantaneous angular error alongwith
    the PID parameters(PID params(Kp, ki, kd), intg and last_errors) into the PID controller(Only P being used here as of now).
Example call: ang_vel = getAngVel(angle_error, params, thresh, intg, last_error)
'''
def getAngVel(error, const, thresh, intg_params, last_error_params):
    ang_vel=0

    # if angular error more than the threshold then computer velocity
    if abs(error) > thresh['angular']:
        if error > 3.14:
            ang_vel = pid((error-6.28), const['angular'], intg_params['w'], last_error_params['w']) # from intg_params and last_error_params choose the intg and last_param 
        elif error < -3.14:                                                                         # meant for angular pid(w)
            ang_vel = pid((error+6.28), const['angular'], intg_params['w'], last_error_params['w'])
        else:
            ang_vel = pid(error, const['angular'], intg_params['w'], last_error_params['w'], )

    return ang_vel

'''
Function Name: getLinearVel
Input: error along x axis, error along y axis, PID params, threshold dictionary, intg, last_error params
Output: linear velocity(along x and y axes)
Logic: 
    This function computes the intantaneous linear velocity along x and y axes using the PID controller. The PID controller takes in the linear error and
    parameters specific to linear PID to compute the velocity
Example call: v_x, v_y = getLinearVel(error_x,  error_y, params, thresh, intg, last_error)
'''
def getLinearVel(error_x,  error_y, const, thresh, intg_params, last_error_params):
    v_x=0
    v_y=0
    
    if abs(error_x)>thresh['linear'] or abs(error_y)>thresh['linear']:
        v_x=pid(error_x, const['linear'], intg_params['vx'], last_error_params['vx'])
        v_y=pid(error_y, const['linear'], intg_params['vy'], last_error_params['vy'])

    return v_x, v_y

'''
Function Name: visualiseTrajectory
Input: list of pixel coordinates that the bots's center has passed through while its pen is down([[x, y], [], [], ....., []])
Output: No output
Logic: 
    This function generates a white, 500x500 px image, iterates the list and blackens out each pixel which helps us visualise the image drawn or function
    plotted by the robot without actually putting a physical pen(This function is just for debugging purpose)
Example call: visualiseTrajectory(trajectory)
'''
def visualiseTrajectory(trajectory):
    frame=np.ones([500,500,3])

    for i in range(len(trajectory)):

        # blacken out the traversed pixels
        frame[int(trajectory[i][1])][int(trajectory[i][0])]=0

        cv2.imshow('trajectory', frame)
        cv2.waitKey(1)
        rospy.sleep(5/len(trajectory))

    cv2.destroyAllWindows()
    