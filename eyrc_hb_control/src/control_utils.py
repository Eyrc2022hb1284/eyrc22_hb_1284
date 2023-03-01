import numpy as np
import rospy
import cv2

# angular pid function
def getAngVel(error, const, ang_thresh, intg_params, last_error_params):
    ang_vel=0

    if abs(error) > ang_thresh:
        if error > 3.14:
            ang_vel = pid((error-6.28), const, intg_params['w'], last_error_params['w'])
        elif error < -3.14:
            ang_vel = pid((error+6.28), const, intg_params['w'], last_error_params['w'])
        else:
            ang_vel = pid(error, const, intg_params['w'], last_error_params['w'], )

        # if ang_vel<0: ang_vel=-3
        # else: ang_vel=3

    return ang_vel

# linear pid function
def getLinearVel(error_x,  error_y, const, linear_thresh, intg_params, last_error_params):
    v_x=0
    v_y=0
    
    if abs(error_x)>linear_thresh or abs(error_y)>linear_thresh:
        v_x=pid(error_x, const, intg_params['vx'], last_error_params['vx'])
        v_y=pid(error_y, const, intg_params['vy'], last_error_params['vy'])

    return v_x, v_y

def pid(error, const, intg, last_error):
        prop = error
        intg = error + intg
        diff = error - last_error
        balance = const['Kp'] * prop + const['Ki'] * intg + const['Kd'] * diff
        last_error = error

        return balance

def visualiseTrajectory(trajectory):
    frame=np.ones([500,500,3])

    for i in range(len(trajectory)):

        # blacken out the traversed pixels
        frame[int(trajectory[i][1])][int(trajectory[i][0])]=0

        cv2.imshow('trajectory', frame)
        cv2.waitKey(1)
        rospy.sleep(5/len(trajectory))

    cv2.destroyAllWindows()
    