#!/usr/bin/env python3

'''
Author: Debrup, Sachin
Purpose: Transforms the camera feed to generate the final processed feed(hb/cam_feed)
'''

import rospy
import cv2
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_utils import transformCorner
import numpy as np

class PerspectiveTransform:
    def __init__(self):
        rospy.init_node('perspective_transform')

        self.bridge=CvBridge()
        self.transfrom_img=Image()
        
        # aruco data
        self.corner_msg=[]
        self.id_msg=[]
        self.aruco_ids=[1, 5, 4, 6] #aruco ids present in the arena corners

        # Perspective Transform params
        self.upperRight=None
        self.upperLeft=None
        self.bottomRight=None
        self.bottomLeft=None

        # subcsribers/publishers
        self.undist_sub=rospy.Subscriber('hb/image_undist', Image, self.img_callback)
        self.corner_sub=rospy.Subscriber('hb/aruco_corners', Int32MultiArray, self.corner_callback)
        self.id_sub=rospy.Subscriber('hb/aruco_id', Int32MultiArray, self.id_callback)
        self.pt_pub=rospy.Publisher('hb/cam_feed', Image, queue_size=10)


    def corner_callback(self, data):
        self.corner_msg=list(data.data)
        self.corner_msg=transformCorner(self.corner_msg)

        # ignore the message if less than 4 aruco markers are detected
        if len(self.corner_msg)>0 and len(self.corner_msg)>0 and len(self.corner_msg)==len(self.id_msg): 
            
            # extract arena corners
            for i in range(len(self.corner_msg)):
                if(self.id_msg[i]==self.aruco_ids[0]): self.bottomRight=self.corner_msg[i][2]

                if(self.id_msg[i]==self.aruco_ids[1]): self.bottomLeft=self.corner_msg[i][3]

                if(self.id_msg[i]==self.aruco_ids[2]): self.upperRight=self.corner_msg[i][1]

                if(self.id_msg[i]==self.aruco_ids[3]): self.upperLeft=self.corner_msg[i][0]

    def id_callback(self, data):
        self.id_msg=list(data.data)

    def img_callback(self, data):
        feed=self.bridge.imgmsg_to_cv2(data, 'bgr8')

        if self.upperLeft is not None and self.upperRight is not None and self.bottomLeft is not None and self.bottomRight is not None: 
            arena_corners=np.float32([self.upperLeft, self.upperRight, self.bottomLeft, self.bottomRight])
            final_feed_corners=np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])

            # perspective transform
            mat=cv2.getPerspectiveTransform(arena_corners, final_feed_corners)
            feed=cv2.warpPerspective(feed, mat, (500, 500))

            self.transfrom_img=self.bridge.cv2_to_imgmsg(feed, 'bgr8')
            self.pt_pub.publish(self.transfrom_img)
            
            cv2.imshow('final_frame', feed)
            cv2.waitKey(1)

            rospy.loginfo("Publishing feed...")

if __name__=='__main__':
    pt=PerspectiveTransform()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

