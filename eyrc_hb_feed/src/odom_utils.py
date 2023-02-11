import cv2
import rospy
import math

#detects the presence of aruco markers in the cam feed and returns the coordinates of the corners
def detect_aruco(aruco_frame, dict, params):
    #detect the markers in the frame
    corners, _, _ = cv2.aruco.detectMarkers(aruco_frame, dict, parameters=params)
    
    return corners


def getPose(corners):
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

		# cv2.waitKey(1)
		
		# orientation
		theta=math.atan2((y-y_rm), (x_rm-x))
		
		return x, y, theta