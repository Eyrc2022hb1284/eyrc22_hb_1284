#!/usr/bin/env python3

'''
Author: Debrup
This script takes up velocities published by controller.py on hb/cmd_vel, converts them into rpm and then transmits it to the HolA bot
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
        self.servo_angle = 90

        self.sub = rospy.Subscriber('hb/cmd_vel', Twist, self.callback)
        rospy.Subscriber('/penStatus', Int32, self.callback_servo)


        # socket object
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while not rospy.is_shutdown():
            
            msg = "{},{},{},{}\r".format(self.fw_rpm,self.rw_rpm,self.lw_rpm,self.servo_angle)
            self.sock.sendto(str.encode(msg), (args.ip, args.port))
            print("Data sent: {}".format(msg))
            
            rate.sleep()

    def callback(self, data):
        fw_vel, lw_vel, rw_vel=getWheelVel(data.linear.x, data.linear.y, data.angular.z, d=0.105, r=0.029)
        # convert m/s to rpm
        self.fw_rpm, self.lw_rpm, self.rw_rpm=Vel2RPM(fw_vel, lw_vel, rw_vel) 


    def callback_servo(self, msg):
        if msg.data == 1:
            self.servo_angle=85
        else:
            self.servo_angle=150      

if __name__ == '__main__':

    # pass ip and port as argument
    parser = argparse.ArgumentParser()
    parser.add_argument('ip', nargs='?', type=str, help='ip address')
    parser.add_argument('-p', '--port', type=int, default=4210, help='port, default: 4210')
    args = parser.parse_args()

    rospy.loginfo("host: %s, port: %s", args.ip, args.port)

    tm = Transmitter(args)
    
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

