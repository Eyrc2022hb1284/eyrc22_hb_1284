#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup
Filename: aruco_detect.py
Theme: HoLA Bot
Functions: img_callback()
Global Variables: None
'''

import rospy
import cv2
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from feed_utils import detect_aruco
import numpy as np

class ArucoDetect:
    def __init__(self):
        rospy.init_node('detect_aruco')

        # instantiate rosmsgs
        self.bridge=CvBridge()
        self.corner_msg=Int32MultiArray()
        self.id_msg=Int32MultiArray()

        # feed param
        self.undist_frame=None

        # load the dictionary that was used to generate the markers
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

        # initializing the detector parameters with default values
        self.params =  cv2.aruco.DetectorParameters_create()

        # subscribers/publishers
        self.sub=rospy.Subscriber('hb/image_undist', Image, self.img_callback)
        self.corner_pub=rospy.Publisher('hb/aruco_corners', Int32MultiArray, queue_size=10)
        self.id_pub=rospy.Publisher('hb/aruco_id', Int32MultiArray, queue_size=10)

        while not rospy.is_shutdown():
            if self.undist_frame is None: continue

            # extract ids and corners of detected aruco markers
            id, corners=detect_aruco(self.undist_frame, self.dict, self.params)
            
            self.id_msg.data=id
            self.corner_msg.data=corners

            # publish data
            self.id_pub.publish(self.id_msg)
            self.corner_pub.publish(self.corner_msg)

            rospy.loginfo('Publishing corners and IDs')
    '''
    Function Name: img_callback
    Input: imgmsg data
    Output: No output
    Logic: 
        This function takes in the image data in rosmsg format, converts it into a cv2 image using CvBridge() and stores it in self.undist_frame
    Example call: self.img_callback(data)
    '''
    def img_callback(self, data):
        self.undist_frame=self.bridge.imgmsg_to_cv2(data)


if __name__=='__main__':
    ad=ArucoDetect()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
