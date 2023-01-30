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
        self.pub=rospy.Publisher('/image_raw', Image, queue_size=10)

        # video object
        vid=cv2.VideoCapture(2, cv2.CAP_V4L)
                
        if  not vid.isOpened():
            print("Camera unavailable")
        else:
            while not rospy.is_shutdown():
                _, frame=vid.read()
                cv2.imshow("frame", frame)

                self.img_msg=self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.pub.publish(self.img_msg)

                rospy.loginfo("Camera feed publishing")

                if cv2.waitKey(27) & 0xFF == ord('q'):
                    rospy.signal_shutdown('user command')

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


