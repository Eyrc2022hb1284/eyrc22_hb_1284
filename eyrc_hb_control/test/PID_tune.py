#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup, Sachin
Filename: controller.py
Theme: HoLA Bot
Functions: goal_callback(), odom_callback(), task_stat_callback(), stop() 
Global Variables: None
'''

# add scripts to system path
import sys
sys.path.insert(0, '/home/kratos/cyborg_ws/src/eyrc_2022_hb/eyrc_hb_control/scripts')

import rospy
from geometry_msgs.msg import Twist
from control_utils import * 
from std_msgs.msg import Float64, Int32
from eyrc_hb_feed.msg import aruco_data
import argparse
from dynamic_reconfigure.server import Server
from eyrc_hb_control.cfg import pidLinearConfig, pidAngularConfig

class goToPose:
    def __init__(self, args):
        # initalise node
        rospy.init_node('PID_testing')

        # task status variable
        self.task_status=1

        # variable to store data regarding feed availability(1->available, -1->Unavailable)
        self.image_availability = 1

        # aruco_data params
        self.odom={'x': 0, 'y': 0, 'theta': 0}

        # current goal
        self.goal={'x': None, 'y': None, 'theta': None}

        # goals
        self.x_goals=[150, 350, 250, 300, 275, 287, 281, 284]
        self.y_goals=[150, 350, 250, 300, 275, 287, 281, 284]
        self.theta_goals=[0, 3.142, 1.571, 2.356, 1.963, 2.159, 2.061, 2.110]

        # tolerance/threshold params
        self.thresh={'linear': args.linearThresh, 'angular': args.angularThresh}    

        # variable that stores the server object handling dynamic reconfiguration
        self.srv = None 

        # initial PID parameters
        self.params={'linear':{'Kp':0.0487, 'Ki':0, 'Kd':0},
                     'angular':{'Kp':5, 'Ki':0, 'Kd':0}}
        # variables for storing integral and last error values 
        self.intg={'vx':0, 'vy':0, 'w':0}
        self.last_error={'vx':0, 'vy':0, 'w':0}
        
        # ros rate
        self.rate=rospy.Rate(75)

        if args.mode == 'l':
            self.srv = Server(pidLinearConfig, self.lin_dyn_callback)
        elif args.mode == 'a':
            self.srv = Server(pidAngularConfig, self.ang_dyn_callback)
        else:
            print("...Invalid mode selected...")
            rospy.signal_shutdown("Invalid mode")

        # subscriber/publisher
        self.odom_sub=rospy.Subscriber('/detected_aruco', aruco_data, self.odom_callback)
        self.task_sub=rospy.Subscriber('/taskStatus', Int32, self.task_stat_callback)
        self.feed_check=rospy.Subscriber('hb/image_raw_check', Int32, self.cam_check_callback)

        # publish error
        self.x_error_pub=rospy.Publisher('hb/error_x', Float64, queue_size=10)
        self.y_error_pub=rospy.Publisher('hb/error_y', Float64, queue_size=10)
        self.theta_error_pub=rospy.Publisher('hb/error_theta', Float64, queue_size=10)

        # publish velocity
        self.twist_pub=rospy.Publisher('hb/cmd_vel', Twist, queue_size=10)           

        # Twist rosmsg
        self.twist_msg=Twist()

        while not rospy.is_shutdown():
            # control loop
            # iterate through each point
            for i in range(len(self.x_goals)):
                # move hola to the desired point
                if args.mode=='l': self.moveHolaTo(self.x_goals[i], self.y_goals[i], 0)
                else: self.moveHolaTo(250, 250, self.theta_goals[i])

    '''
    Function Name: lin_dyn_callback
    Input: current configuration(config), level
    Output: updated configuration
    Logic: 
        This function takes in the incoming config and level data, updates the linear PID parameters 
        and returns the updated configuration
    Example call: self.lin_dyn_callback(config, level)
    '''      
    def lin_dyn_callback(self, config, level):
       
        # update parameters
        self.params['linear']['Kp'] = config['Kp_l']
        self.params['linear']['Ki'] = config['Ki_l']
        self.params['linear']['Kd'] = config['Kd_l']

        return config

    '''
    Function Name: ang_dyn_callback
    Input: current configuration(config), level
    Output: updated configuration
    Logic: 
        This function takes in the incoming config and level data, updates the angular PID parameters 
        and returns the updated configuration
    Example call: self.ang_dyn_callback(config, level)
    '''     
    def ang_dyn_callback(self, config, level):
       
        # update parameters
        self.params['angular']['Kp'] = config['Kp_a']
        self.params['angular']['Ki'] = config['Ki_a']
        self.params['angular']['Kd'] = config['Kd_a']

        return config

    '''
    Function Name: odom_callback
    Input: odometry data(x, y, theta)
    Output: No output
    Logic: 
        This function takes in the incoming odometry message from the /detected_aruco topic and stores it 
        into the odom dictionary.
    Example call: self.odom_callback(data)
    '''       
    def odom_callback(self, msg):
        self.odom['x']=msg.x 
        self.odom['y']=msg.y
        self.odom['theta']=msg.theta

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
    Function Name: cam_check_callback
    Input: Int data
    Output: No output
    Logic: 
        This function takes in the incoming int data that signifies if camera feed is available or not and stops the 
        robot incase camera is unavailable
    Example call: self.cam_check_callback(data)
    '''    
    def cam_check_callback(self, data):
        self.image_availability=data.data

		# if no image is getting published
        if self.image_availability == -1:
            rospy.loginfo("Feed unavailable")


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
        self.goal['x']=x
        self.goal['y']=y
        self.goal['theta']=theta

        print("Goal: [{}, {}, {}]".format(x, y, theta))

        # move the bot untill it reaches the goal point
        while True:
            # if aruco marker isn't detected or task status = 1, stop the bot
            if (self.odom['x']==-1 and self.odom['y']==-1 and self.odom['theta']==4) or self.task_status==1 or self.image_availability==-1:
                self.stop()
            else:
                # error calculation
                angle_error, error_x, error_y = calculateError(self.goal, self.odom)

                # velocity calculation
                v_x, v_y=getLinearVel(error_x,  error_y, self.params, self.thresh, self.intg, self.last_error)
                ang_vel=getAngVel(angle_error, self.params, self.thresh, self.intg, self.last_error)

                # setup the msg for publishing
                self.twist_msg.linear.x=v_x 
                self.twist_msg.linear.y=v_y
                self.twist_msg.angular.z=ang_vel

                # publish error
                self.x_error_pub.publish(error_x)
                self.y_error_pub.publish(error_y)
                self.theta_error_pub.publish(angle_error)

                # publish onto hb/cmd_vel
                self.twist_pub.publish(self.twist_msg)
                self.rate.sleep()

                #stop when reached target pose
                if abs(angle_error)<=self.thresh['angular'] and abs(error_x)<=self.thresh['linear'] and abs(error_y)<=self.thresh['linear']:
                    print("reached goal pose: [{}, {}, {}]".format(self.x,  self.y, round(self.theta, 3)))
                    # self.stop()
                    break

if __name__=='__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', "--mode", nargs='?', type=str, help='mode: l-linear PID, a-angular PID')
    parser.add_argument('-l', "--linearThresh", default=0, type=int, help='linear threshold(in terms of pixels)')
    parser.add_argument('-a', "--angularThresh", default=0, type=float, help='angular threshold(in terms of radians)')
    
    args = parser.parse_args()

    gt=goToPose(args)
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)