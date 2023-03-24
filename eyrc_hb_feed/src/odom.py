#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Debrup
Filename: odom.py
Theme: HoLA Bot
Functions: callback()
Global Variables: None
'''

import rospy 				
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2				
from feed_utils import detect_aruco, getPose
from std_msgs.msg import Int32
from eyrc_hb_feed.msg import aruco_data

class Odom:
	def __init__(self):
		# initialise node
		rospy.init_node('odom')  

		# rosmsgs
		self.bridge=CvBridge()
		self.pose_msg = aruco_data()

		# load the dictionary that was used to generate the markers
		self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

		# initializing the detector parameters with default values
		self.params =  cv2.aruco.DetectorParameters_create()

		# feed frame
		self.frame=None
		self.image_availability=1

		# bot aruco marker id
		self.botID=15

		# variable that stores all corners of detected aruco markers
		self.bot_aruco_corners=[]
		
		self.feed = rospy.Subscriber('hb/cam_feed', Image, self.callback)
		self.feed_check=rospy.Subscriber('hb/image_raw_check', Int32, self.cam_check_callback)
		self.pub = rospy.Publisher('/detected_aruco', aruco_data, queue_size=10)

		while not rospy.is_shutdown():
			if self.frame is None: continue

			# make a copy(this copy is used for displaying current orientation)
			frame=self.frame.copy()
			# detect ids and corners
			id, corners=detect_aruco(self.frame, self.dict, self.params)
			
			# if no aruco marker detected
			if len(corners)==0 or len(id)==0 or len(corners)!=len(id): 
				self.pose_msg.x=-1
				self.pose_msg.y=-1
				self.pose_msg.theta=4
				self.pub.publish(self.pose_msg)
			# else, calculate pose
			else:
				# get bot aruco corners
				for i in range(len(corners)):
					if id[i][0]==self.botID:
						self.bot_aruco_corners=corners[i][0]
						break

				# if bot aruco marker corners not found, continue to try detecting
				if len(self.bot_aruco_corners)==0: continue
					
				# store pose in the pose msg
				self.pose_msg.x, self.pose_msg.y, self.pose_msg.theta = getPose(self.bot_aruco_corners)

			# display orientaton data in frame
			frame = cv2.putText(frame, 'x: {} y: {} theta: {}'.format(self.pose_msg.x, self.pose_msg.y, round(self.pose_msg.theta, 3)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
			
			# display perspectively transmformed image for visual references
			cv2.imshow('frame', frame)
			cv2.waitKey(1)

			# publish odometry data onto 'aruco_data' rosmsg
			self.pub.publish(self.pose_msg)
			rospy.loginfo("Publishing Odom")
			
	'''
	Function Name: callback
	Input: Image rosmsg data
	Output: None
	Logic: 
		This function takes in the incoming feed from the rostopic 'hb/cam_feed' and stores it into self.frame
	Example call: self.callback(data)
	''' 
	def callback(self, data):
		# recieves the feed
		self.frame = self.bridge.imgmsg_to_cv2(data, "mono8")

	def cam_check_callback(self, data):
		self.image_availability=data.data

		# if no image is getting published
		if self.image_availability == -1:
			rospy.loginfo("Feed unavailable")
			rospy.signal_shutdown("feed unavailable")

if __name__ == '__main__':
    od=Odom()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)