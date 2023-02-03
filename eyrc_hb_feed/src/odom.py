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

		# pose
		self.x=0
		self.y=0
		self.theta=0
		
		rospy.Subscriber('hb/image_undist', Image, self.callback)
		self.pub = rospy.Publisher('hb/odom', Pose2D, queue_size=10)

		while not rospy.is_shutdown():
			if self.frame is None: continue

			corners=self.detect_aruco(self.frame)
			if len(corners)==0: continue

			# store pose in the pose msg
			self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = self.getPose(corners[0][0])
			self.pub.publish(self.pose_msg)
			rospy.loginfo("Publishing Odom")

	def callback(self, data):
		# recieves the feed
		rospy.loginfo("receiving camera frame")
		get_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
		self.frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

    #detects the presence of aruco markers in the cam feed and returns the coordinates of the corners
	def detect_aruco(self, aruco_frame):
        #detect the markers in the frame
		corners, _, _ = cv2.aruco.detectMarkers(aruco_frame, self.dict, parameters=self.params)
		
		return corners

	def getPose(self, corners):
		# returns pose of the bot

		x1, y1=corners[0] #upper left
		x2, y2=corners[2] #lower right
		x3, y3=corners[1] #upper right

		#mid point of aruco marker
		x, y=[int((x1+x2)/2), int((y1+y2)/2)]       
		#mid point of right side of aruco marker  
		x_rm, y_rm=[int((x2+x3)/2), int((y2+y3)/2)]   

		# frame=cv2.arrowedLine(self.frame, (x, y), (x_rm, y_rm), (255, 0, 255), 1, 8, 0, 0.1)
		# cv2.imshow('frame', frame)

		if cv2.waitKey(27) & 0xFF == ord('q'):
			rospy.signal_shutdown('user command')
		
		# orientation
		theta=math.atan2((y-y_rm), (x_rm-x))
		
		return x, y, theta

if __name__ == '__main__':
    od=Odom()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)