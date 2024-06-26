#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup
Filename: hola_transmitter.py
Theme: HoLA Bot
Functions: callback(), callback_servo()
Global Variables: None
'''

import argparse
import socket
from communication_utils import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class Transmitter:
    def __init__(self, args):
        rospy.init_node('transmitter')

        # rate
        rate = rospy.Rate(75)

        # variables that store RPM
        self.fw_rpm=0
        self.lw_rpm=0
        self.rw_rpm=0

        # pen up angle
        self.servo_angle = {'image': 4, 'function': 4}

        # subscribers
        self.sub = rospy.Subscriber('hb/cmd_vel', Twist, self.callback)
        self.pen_status_sub = {'image': rospy.Subscriber('/img_penStatus', Int32, self.img_pen_callback),
                               'function': rospy.Subscriber('/func_penStatus', Int32, self.func_pen_callback)}

        # socket object
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while not rospy.is_shutdown():
            
            # make the message
            msg = "{},{},{},{},{}\r".format(self.fw_rpm,self.rw_rpm,self.lw_rpm,self.servo_angle['image'],self.servo_angle['function'])
            # transmit data
            self.sock.sendto(str.encode(msg), (args.ip, args.port))
            print("Data sent: {}".format(msg))
            
            rate.sleep()

    '''
    Function Name: callback
    Input: Twist data
    Output: None
    Logic: 
        This function gets the chassis velocity, calculates the wheel velocities (calls the getWheelVel method) and stores them in a variable
    Example call: self.callback(data)
    '''
    def callback(self, data):
        fw_vel, lw_vel, rw_vel=getWheelVel(data.linear.x, data.linear.y, data.angular.z, d=0.105, r=0.029)
        # convert velocities to rpm
        self.fw_rpm, self.lw_rpm, self.rw_rpm=Vel2RPM(fw_vel, lw_vel, rw_vel) 

    '''
    Function Name: img_pen_callback
    Input: Int data
    Output: None
    Logic: 
        This function gets the status of the pen responsible for drawing the image, and stores servo angle to be given to the image servo in the variable accordingly
    Example call: self.img_pen_callback(data)
    '''
    def img_pen_callback(self, msg):
        # pen down
        if msg.data == 1:
            self.servo_angle['image']=4
        # pen up
        else:
            self.servo_angle['image']=10

    '''
    Function Name: func_pen_callback
    Input: Int data
    Output: None
    Logic: 
        This function gets the status of the pen responsible for plotting the function, and stores servo angle to be given to the function servo in the variable accordingly
    Example call: self.func_pen_callback(data)
    '''
    def func_pen_callback(self, msg):
        # pen down
        if msg.data == 1:
            self.servo_angle['function']=4
        # pen up
        else:
            self.servo_angle['function']=10

if __name__ == '__main__':

    # pass ip and port number as argument
    parser = argparse.ArgumentParser()
    parser.add_argument('ip', nargs='?', type=str, help='ip address')
    parser.add_argument('-p', '--port', type=int, default=4210, help='port, default: 4210')
    args = parser.parse_args()
    
    tm = Transmitter(args)
    
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

