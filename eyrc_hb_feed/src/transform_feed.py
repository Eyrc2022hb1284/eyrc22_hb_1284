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
from feed_utils import detect_aruco, extract_arena_corners, perspectiveTransform
import numpy as np

class PerspectiveTransform:
    def __init__(self):
        rospy.init_node('perspective_transform')

        self.bridge=CvBridge()
        self.transfrom_img=Image()
        
        # aruco data
        self.corners=[]
        self.ids=[]
        self.aruco_ids=[12, 4, 10, 8] #aruco ids present in the arena corners

        # Perspective Transform params
        self.upperRight=[397, 86]
        self.upperLeft=[98, 88]
        self.bottomRight=[395, 430]
        self.bottomLeft=[98, 424]

        # feed params
        self.feed=None
        self.final_feed_corners=np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])

        # load the dictionary that was used to generate the markers
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

        # initializing the detector parameters with default values
        self.params =  cv2.aruco.DetectorParameters_create()

        # subcsribers/publishers
        self.undist_sub=rospy.Subscriber('hb/image_undist', Image, self.img_callback)
        self.pt_pub=rospy.Publisher('hb/cam_feed', Image, queue_size=10)

    def img_callback(self, data):
        self.feed=self.bridge.imgmsg_to_cv2(data, 'bgr8')

        self.ids, self.corners=detect_aruco(self.feed, self.dict, self.params)

        # if len(self.corners)!=0 and len(self.ids)!=0 and len(self.corners)==len(self.ids):
        
        curr_upperRight, curr_upperLeft, curr_bottomRight, curr_bottomLeft=extract_arena_corners(self.corners, self.ids, self.aruco_ids)

        # dont update corners if they arent detected
        if(curr_upperRight is not None): self.upperRight=curr_upperRight
        if(curr_upperLeft is not None): self.upperLeft=curr_upperLeft
        if(curr_bottomRight is not None): self.bottomRight=curr_bottomRight
        if(curr_bottomLeft is not None): self.bottomLeft=curr_bottomLeft

        if self.upperLeft is not None and self.upperRight is not None and self.bottomLeft is not None and self.bottomRight is not None:

            # print(self.upperRight, self.upperLeft, self.bottomRight, self.bottomLeft)

            arena_corners=np.float32([self.upperLeft, self.upperRight, self.bottomLeft, self.bottomRight])
            
            # perspective transform
            self.feed=perspectiveTransform(self.feed, arena_corners, self.final_feed_corners)

            self.transfrom_img=self.bridge.cv2_to_imgmsg(self.feed, 'bgr8')
            self.pt_pub.publish(self.transfrom_img)
            
            rospy.loginfo("Publishing feed...")

if __name__=='__main__':
    pt=PerspectiveTransform()

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

