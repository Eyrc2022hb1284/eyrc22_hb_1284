#!/usr/bin/env python3

'''
Author: Debrup
Purpose: generate waypoints/goals to draw the given image/function
'''

import rospy
import argparse
from std_msgs.msg import String
from feed_utils import getImage, getContourMsg

class getCountour:
    def __init__(self, args):
        # initiate node
        rospy.init_node('get_contours')

        # ros rate
        self.rate=rospy.Rate(1)
        # publisher
        self.pub=rospy.Publisher('/contours', String, queue_size=10)
        
        # stores the waypoints/goalpoints
        self.contour_msg=None

        # if image mode
        if args.mode == 0:

            # generate image
            image=getImage(args.name)
            # generate waypoints
            self.contour_msg=getContourMsg(mode=args.mode, image=image, density=args.density)

        # if function mode
        elif args.mode == 1:
            self.contour_msg=getContourMsg(mode=args.mode, points=args.points)

        # invalid mode selected
        else:
            rospy.loginfo("Imvalid mode!")
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
    parser.add_argument('-p', '--points', type=int, default=500, help='no.of goal points it the generated waypoint list')
    parser.add_argument('-d', '--density', type=int, default=3, help='gap in terms of pixels between adjacent waypoints')
    args = parser.parse_args()

    gc = getCountour(args)
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)