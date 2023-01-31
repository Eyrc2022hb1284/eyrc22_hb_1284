#!/usr/bin/env python3

'''
Author: Debrup, Sachin
Purpose: Publish hb/undist_feed from hb/image_raw 
'''

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle

class FeedUndistort:
    def __init__(self):
        rospy.init_node('undistort_feed')

        self.bridge=CvBridge()
        self.image_msg=Image()
        self.image=None
        self.dist_param=None

        self.sub=rospy.Subscriber('hb/image_raw', Image, self.cam_callback)
        self.pub=rospy.Publisher('hb/image_undist', Image, queue_size=10)

        with open("eyrc_hb_feed/params/undist_params.p", 'rb') as file:
            self.dist_param = pickle.load(file)


    def cam_callback(self, data):
        self.image=self.bridge.imgmsg_to_cv2(data)
        
        undistorted_frame = cv2.undistort(self.image, self.dist_param["mtx"], self.dist_param["dist"], None, self.dist_param["mtx"])
        self.image_msg = self.bridge.cv2_to_imgmsg(undistorted_frame, 'bgr8')
        self.pub.publish(self.image_msg)

        rospy.loginfo("Feed undistorting...")

        cv2.waitKey(1)
        


if __name__=='__main__':
    fu=FeedUndistort()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

