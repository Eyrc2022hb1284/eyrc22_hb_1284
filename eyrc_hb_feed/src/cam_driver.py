#!/usr/bin/env python3

'''
Author: Debrup
Purpose: Publish /image_raw from camera feed
'''

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamDriver:
    def __init__(self):
        rospy.init_node("cam_driver")

        # bridge object
        self.bridge=CvBridge()
        # publisher
        self.pub=rospy.Publisher('hb/image_raw', Image, queue_size=10)

        # video object
        vid=cv2.VideoCapture(2, cv2.CAP_V4L)

        while not rospy.is_shutdown():

            if  not vid.isOpened():
                rospy.loginfo("Cannot access camera")
                rospy.signal_shutdown("Camera unavailable")
            else:
                _, frame=vid.read()
                self.img_msg=self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.pub.publish(self.img_msg)
                rospy.loginfo("camera feed publishing")

        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()

if __name__=='__main__':
    cd=CamDriver()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)


