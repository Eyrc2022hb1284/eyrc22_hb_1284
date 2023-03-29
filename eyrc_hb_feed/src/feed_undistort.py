#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup, Sachin
Filename: feed_undist.py
Theme: HoLA Bot
Functions: cam_callback()
Global Variables: None
'''

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import pickle

class FeedUndistort:
    def __init__(self):
        rospy.init_node('undistort_feed')

        # cvbridge object
        self.bridge=CvBridge()
        # image rosmsg
        self.image_msg=Image()

        # variable to store feed 
        self.image=None
        self.image_availability=1

        # variable to store the undistortion parameters
        self.dist_param=None

        # subscriber/publisher
        self.feed=rospy.Subscriber('hb/image_raw', Image, self.cam_callback)
        self.feed_check=rospy.Subscriber('hb/image_raw_check', Int32, self.cam_check_callback)

        self.pub=rospy.Publisher('hb/image_undist', Image, queue_size=10)

        # load undistorting parameters
        with open("/home/kratos/cyborg_ws/src/eyrc_2022_hb/eyrc_hb_feed/params/undist_params.p", 'rb') as file:
            self.dist_param = pickle.load(file)            


    '''
    Function Name: cam_callback
    Input: imgmsg data
    Output: No output
    Logic: 
        This function takes in the image data in rosmsg format, converts it into a cv2 image using CvBridge() and stores it in self.image
    Example call: self.cam_callback(data)
    '''
    def cam_callback(self, data):
        self.image=self.bridge.imgmsg_to_cv2(data)
        
        # undistort
        undistorted_frame = cv2.undistort(self.image, self.dist_param["mtx"], self.dist_param["dist"], None, self.dist_param["mtx"])
        
        # publish undistorted feed
        self.image_msg = self.bridge.cv2_to_imgmsg(undistorted_frame, 'bgr8')
        self.pub.publish(self.image_msg)
        rospy.loginfo("camera feed undistorting...")

    def cam_check_callback(self, data):
        self.image_availability=data.data

        # if no image is getting published
        if self.image_availability == -1:
            rospy.loginfo("Feed unavailable")
            rospy.signal_shutdown("feed unavailable")

        
if __name__=='__main__':
    fu=FeedUndistort()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

