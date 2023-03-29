#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup
Filename: getContours.py
Theme: HoLA Bot
Functions: None
Global Variables: None
'''

import rospy
import argparse
from std_msgs.msg import String
from feed_utils import getImage, getContourMsg, verifyArgs
import cv2
import ast

class getCountour:
    def __init__(self, args):
        # initiate node
        rospy.init_node('get_contours_{}'.format(args.mode))

        # ros rate
        self.rate=rospy.Rate(1)
        # publisher
        self.pub=rospy.Publisher('/contours_{}'.format(args.mode), String, queue_size=10)
        
        # stores the waypoints/goalpoints
        self.contour_msg=None

        # verify all arguments
        verifyArgs(args)
        
        # if image mode
        if args.mode == 0:
            # generate image
            image=getImage(args.name, ast.literal_eval(args.frameSize))
            
            # generate waypoints
            self.contour_msg=getContourMsg(mode=args.mode, image=image, density=args.density, frame_size=ast.literal_eval(args.frameSize), frame_start=ast.literal_eval(args.frameStart))

        # else if function mode
        elif args.mode == 1:
            self.contour_msg=getContourMsg(mode=args.mode, points=args.points, frame_size=ast.literal_eval(args.frameSize), frame_start=ast.literal_eval(args.frameStart))

        # invalid mode selected
        else:
            rospy.loginfo("Invalid mode!")
            rospy.signal_shutdown("Wrong user input")

        while not rospy.is_shutdown():
            # publish message
            self.pub.publish(str(self.contour_msg))
            rospy.loginfo("Waypoints published!")
            self.rate.sleep()


if __name__=='__main__':

    # pass image to be drawn as argument
    # mode 0-Image mode:
        # input-mode, image, pixel density
    # mode 1-Function mode
        # input-mode, no.of points 
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', "--mode", nargs='?', type=int, help='mode: 0-image, 1-function')
    parser.add_argument('-n', '--name', type=str, default="", help='name of image file to be drawn')
    parser.add_argument('-f', '--frameSize', type=str, default='(500, 500)', help='frame dimensions((x, y)-> x by y sized frame)')
    parser.add_argument('-s', '--frameStart', type=str, default='(0, 0)', help='upper left corner of the frame')
    parser.add_argument('-p', '--points', type=int, default=500, help='no.of goal points it the generated waypoint list')
    parser.add_argument('-d', '--density', type=int, default=3, help='gap in terms of pixels between adjacent waypoints')
    args = parser.parse_args()

    gc = getCountour(args)
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)