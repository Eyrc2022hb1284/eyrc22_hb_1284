import cv2
import math
import numpy as np

#detects the presence of aruco markers in the cam feed and returns the coordinates of the corners
def detect_aruco(aruco_frame, dict, params):
	#detect the markers in the frame
	corners, id, _ = cv2.aruco.detectMarkers(aruco_frame, dict, parameters=params)
	
	return id, corners

# extract the 4 corners of the arena
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

# performs perspective transform
def perspectiveTransform(feed, arena_corners, final_feed_corners):
	mat=cv2.getPerspectiveTransform(arena_corners, final_feed_corners)
	feed=cv2.warpPerspective(feed, mat, (500, 500))

	return feed

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

		# orientation
		theta=math.atan2((y-y_rm), (x_rm-x))
		
		return x, 500-y, theta

# resize image
def resize(img, w, h):
	return cv2.resize(img, (w, h))

# extract contour coordinates from image/function
def getContourMsg(image=None, points=500, mode=None):
	if mode == 0:
		print("...Image mode selected...")
		contours=getContoursImg(image, points)
	if mode == 1:
		print("..Function mode selected...")
		contours=getContoursFunc(points)

	return contours

# contour extraction algo
def getContoursImg(image, points):
	gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	kernel=cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	eroded=cv2.erode(gray, kernel, iterations=1)
	dilated=cv2.dilate(eroded, kernel, iterations=1)

	cv2.imshow('frame', dilated)
	cv2.waitKey(0)
	return []

# function plotting algo
def getContoursFunc(points):
	t=[0, 2*math.pi]

	t_low=t[0]
	t_high=t[1]

	x_goals=[]
	y_goals=[]
	theta_goals=[]

	for i in range(points):
		curr_t=t_low+i*(t_high-t_low)/points
		x_goal, y_goal, theta_goal=getCurrGoal(curr_t)

		x_goals.append(int(x_goal)+250) #linear transformation
		y_goals.append(500-(int(y_goal)+250)) #linear transformation followed by conversion to openCV corrdinate
		theta_goals.append(theta_goal)

	return [x_goals, y_goals, theta_goals]

# generate goals wrt time
def getCurrGoal(curr_t):
	return 200*math.cos(curr_t), 100*math.sin(2*curr_t), (math.pi/4)*math.sin(curr_t)





