#!/usr/bin/env python3

'''
Author: Debrup
Purpose: Calculates current odometry of the robot and publishes it onto hb/odom rostopic
'''

import rospy 				
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2				
import math	
from geometry_msgs.msg import Pose2D
from aruco_utils import *

class Odom:
	def __init__(self):
		rospy.init_node('odom')  

		self.bridge=CvBridge()
		self.pose_msg = Pose2D()

		# load the dictionary that was used to generate the markers
		self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

		# initializing the detector parameters with default values
		self.params =  cv2.aruco.DetectorParameters_create()

		# feed frame
		self.frame=None
		
		# bot aruco marker id
		self.botID=3
		# pose
		self.x=0
		self.y=0
		self.theta=0
		
		rospy.Subscriber('hb/cam_feed', Image, self.callback)
		self.pub = rospy.Publisher('hb/odom', Pose2D, queue_size=10)

		while not rospy.is_shutdown():
			if self.frame is None: continue

			id, corners=detect_aruco(self.frame, self.dict, self.params)
			corners=transformCorner(corners)
			# print(corners)

			# store pose in the pose msg
			self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = getPose(corners[0])

			self.pub.publish(self.pose_msg)

			rospy.loginfo("Publishing Odom")

	def callback(self, data):
		# recieves the feed
		self.frame = self.bridge.imgmsg_to_cv2(data, "mono8")
		# self.frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

if __name__ == '__main__':
    od=Odom()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)