import cv2
import math
import numpy as np

#detects the presence of aruco markers in the cam feed and returns the coordinates of the corners
def detect_aruco(aruco_frame, dict, params):
	#detect the markers in the frame
	corners, id, _ = cv2.aruco.detectMarkers(aruco_frame, dict, parameters=params)

	# convert to numpy and change dtype
	corners = np.array(corners)
	id = np.array(id)
	corners = corners.astype(np.int32)
	id=id.astype(np.int32)

	# flatten the ndarray
	corners=corners.flatten()

	# convert to list
	corners.tolist()
	id.tolist()

	# explicitly convert dtype
	id=[int(id[j]) for j in range(len(id))]
	corners=[int(corners[j]) for j in range(len(corners))]
	
	return id, corners

def transformCorner(corners):
	corners=np.array(corners)
	# print(corners)
	length=len(corners)
	# print(length)
	rows=int(length/8)
	# print(rows)

	if rows==0 : return []
	else:
		corners=corners.reshape((rows, 4, 2))
		# print(corners)
		return corners


# calcualtes pose of the robot
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