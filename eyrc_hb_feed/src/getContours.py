#!/usr/bin/env python3

'''
Author: Debrup
Purpose: generate waypoints/goals to draw the given image/function
'''

import rospy
import argparse
from std_msgs.msg import String
import cv2
from feed_utils import resize, getContourMsg
import math

class getCountour:
    def __init__(self, args):
        rospy.init_node('get_contours')

        self.rate=rospy.Rate(1)
        self.pub=rospy.Publisher('/contours', String, queue_size=10)
        
        self.contour_msg=None

        # if image mode
        if args.mode == 0:
            path='/home/kratos/cyborg_ws/src/eyrc_2022_hb/eyrc_hb_feed/src/taskImages/{}'.format(args.name)
            # read image
            image=cv2.imread(path)        
            # resize
            image=resize(image, w=500, h=500)
            self.contour_msg=getContourMsg(image=image, points=args.points, mode=args.mode)

        # if function mode
        elif args.mode == 1:
            self.contour_msg=getContourMsg(points=args.points, mode=args.mode)

        else:
            rospy.loginfo("Imvalid mode!")
            rospy.signal_shutdown("Wrong user input")

        while not rospy.is_shutdown():
            # print(self.contour_msg)
            self.pub.publish(str(self.contour_msg))
            rospy.loginfo("Waypoints published!")
            self.rate.sleep()
            # rospy.signal_shutdown("Contours published!")


if __name__=='__main__':

    # pass image to be drawn as argument
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', "--mode", nargs='?', type=int, help='mode: 0-image, 1-function')
    parser.add_argument('-n', '--name', type=str, default="", help='name of image file to be drawn')
    parser.add_argument('-p', '--points', type=int, default=500, help='no.of goal points it the generated contour')
    args = parser.parse_args()

    gc = getCountour(args)
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)