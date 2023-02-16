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
from aruco_utils import detect_aruco
from odom_utils import getPose

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
		self.botID=15
		self.bot_aruco_corners=[]
		
		rospy.Subscriber('hb/cam_feed', Image, self.callback)
		self.pub = rospy.Publisher('hb/odom', Pose2D, queue_size=10)

		while not rospy.is_shutdown():
			if self.frame is None: continue

			id, corners=detect_aruco(self.frame, self.dict, self.params)

			if len(corners)==0 or len(id)==0 or len(corners)!=len(id): 
				self.pose_msg.x=-1
				self.pose_msg.y=-1
				self.pose_msg.theta=4
				self.pub.publish(self.pose_msg)
			
			else:
				# get bot aruco corners
				for i in range(len(corners)):
					if id[i][0]==self.botID:
						self.bot_aruco_corners=corners[i][0]
						break

				# if bot aruco marker corners not found
				if len(self.bot_aruco_corners)==0: continue
					
				# store pose in the pose msg
				self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = getPose(self.bot_aruco_corners)

				self.frame = cv2.putText(self.frame, 'x: {} y: {} theta: {}'.format(self.pose_msg.x, self.pose_msg.y, round(self.pose_msg.theta, 3)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
				cv2.imshow('frame', self.frame)
				cv2.waitKey(1)

			self.pub.publish(self.pose_msg)
			rospy.loginfo("Publishing Odom")

	def callback(self, data):
		# recieves the feed
		self.frame = self.bridge.imgmsg_to_cv2(data, "mono8")


if __name__ == '__main__':
    od=Odom()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)