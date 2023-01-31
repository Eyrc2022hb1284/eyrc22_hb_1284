#!/usr/bin/env python3

import argparse
import telnetlib
import rospy
from geometry_msgs.msg import Twist
from eyrc_hb_control.src.inverse_kinematics import getWheelVel

class Transmitter:
    def __init__(self, args):
        rospy.init_node('transmitter')

        self.telnet = telnetlib.Telnet(args.ip, args.port)
        self.sub = rospy.Subscriber('hb/cmd_vel', Twist, self.callback)

    def callback(self, data):

        fw_vel, lw_vel, rw_vel=getWheelVel(data.linear.x, data.linear.y, data.angular.z, d=0.105, r=0.029)
        msg = "{}, {}, {}, 0\r".format(fw_vel, lw_vel, rw_vel)

        self.telnet.write(msg.encode('ascii'))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, default="192.168.0.1", help='ip address')
    parser.add_argument('-p', '--port', type=int, default="8888", help='port')
    args = parser.parse_args()

    print(args.ip, args.port)
    rospy.loginfo("Connected...IP: {} Port: {}".format(args.ip, args.port))

    tm = Transmitter()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

