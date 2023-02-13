import cv2
import math
import numpy as np

#detects the presence of aruco markers in the cam feed and returns the coordinates of the corners
def detect_aruco(aruco_frame, dict, params):
	#detect the markers in the frame
	corners, id, _ = cv2.aruco.detectMarkers(aruco_frame, dict, parameters=params)
	
	return id, corners

def extract_arena_corners(corner_msg, id_msg, aruco_ids):
	upperLeft=None
	upperRight=None
	bottomLeft=None
	bottomRight=None
	
	for i in range(len(corner_msg)):
		if(id_msg[i][0]==aruco_ids[0]): bottomRight=corner_msg[i][0][2]

		if(id_msg[i][0]==aruco_ids[1]): bottomLeft=corner_msg[i][0][3]

		if(id_msg[i][0]==aruco_ids[2]): upperRight=corner_msg[i][0][1]

		if(id_msg[i][0]==aruco_ids[3]): upperLeft=corner_msg[i][0][0]
	
	return upperRight, upperLeft, bottomRight, bottomLeft

def perspectiveTransform(feed, arena_corners, final_feed_corners):
	mat=cv2.getPerspectiveTransform(arena_corners, final_feed_corners)
	feed=cv2.warpPerspective(feed, mat, (500, 500))

	return feed

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
