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
		
		return x, y, theta

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
	# Convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# Threshold the image
	thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

	# Apply morphological closing operation to fill in gaps in the lines
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
	closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

	# Dilate the image to make the lines thicker
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
	dilated = cv2.dilate(closed, kernel, iterations=1)

	# Find the contours in the image
	contours, hierarchy = cv2.findContours(dilated, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
	
	outer_contours=[]
	# Extract the contours who dont have a parent
	for i in range(len(contours)):
		if hierarchy[0][i][3] == -1:
			outer_contours.append(contours[i])

	# cv2.drawContours(image, outer_contours, -1, (0, 255, 0), 1)
	# cv2.imshow('frame', image)
	# cv2.waitKey(0)

	# extract x and y goals into 2 separate lists
	x_goals, y_goals=splitContours(outer_contours)
	theta_goals=[]

	# generate theta goals
	for i in range(len(x_goals)):
		theta_goals.append([0]*len(x_goals[i]))
	
	return [x_goals, y_goals, theta_goals]

# function plotting algo
def getContoursFunc(points):
	t=[0, 2*math.pi]

	t_low=t[0]
	t_high=t[1]

	contours=[]
	contour=[]
	theta_goals=[]
	single_contour_theta_goals=[]

	for i in range(points):
		curr_t=t_low+i*(t_high-t_low)/points

		# get current pixel
		x_goal, y_goal, theta_goal=getCurrGoal(curr_t)

		x_goal=int(x_goal)+250 #linear transformation
		y_goal=500-(int(y_goal)+250) #linear transformation followed by conversion to openCV corrdinate

		# if contour is empty or incoming pixel is close to the previous one, add them into the same contour
		if len(contour)==0 or isPxNearby(contour[-1][0], [x_goal, y_goal]):
			contour.append([[x_goal, y_goal]])
			single_contour_theta_goals.append(theta_goal)

		# else store the current contour, and insert the pixel into a new contour
		else:
			contours.append[contour]
			theta_goals.append(single_contour_theta_goals)

			contour.clear()
			single_contour_theta_goals.clear()

			contour.append([[x_goal, y_goal]])
			single_contour_theta_goals.append(theta_goal)

	# append the last contour
	contours.append(contour)
	theta_goals.append(single_contour_theta_goals)

	# print(contours)
	# extract x and y goals into 2 separate lists
	x_goals, y_goals=splitContours(contours)

	# print(x_goals, y_goals, theta_goals)
	return [x_goals, y_goals, theta_goals]

# generate goals wrt time
def getCurrGoal(curr_t):
	return 200*math.cos(curr_t), 100*math.sin(2*curr_t), (math.pi/4)*math.sin(curr_t)

# returns a true value if any one pixel coordinate is closer than 10 pixels from the other
def isPxNearby(pixelA, pixelB):
	return abs(pixelA[0]-pixelB[0])<10 and abs(pixelA[1]-pixelB[1])<10

def splitContours(contours):
	x_goals=[]
	y_goals=[]

	# split contours([[[x, y]..,[]],[]...,[]]) to extract x_goals([[x1, x2, ...],[]...,[]) and y_goals([[y1,y2...],[]...,[]])
	for contour in contours:
		x_goal=[]
		y_goal=[]
		for i in range(len(contour)):
			x_goal.append(contour[i][0][0])
			y_goal.append(contour[i][0][1])
		
		x_goals.append(x_goal)
		y_goals.append(y_goal)
	
	return x_goals, y_goals


