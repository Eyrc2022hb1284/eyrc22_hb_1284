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

	# Find the contours in the image
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
	
	waypoints=[]
	# Extract the contours who dont have a parent
	for i in range(len(contours)):
		if hierarchy[0][i][3] == -1:
			waypoints.append(contours[i].tolist())

	# add theta to each pixel to convert it to a waypoint
	waypoints=addTheta(waypoints)

	# extract x, y, theta goals into 3 separate lists
	x_goals, y_goals, theta_goals=splitContours(waypoints)
	
	return [x_goals, y_goals, theta_goals]

# function plotting algo
def getContoursFunc(points):
	t=[0, 2*math.pi]

	t_low=t[0]
	t_high=t[1]

	contours=[]

	for i in range(points):
		curr_t=t_low+i*(t_high-t_low)/points

		# get current pixel
		x_goal, y_goal, theta_goal=getCurrGoal(curr_t)

		x_goal=int(x_goal)+250 #linear transformation
		y_goal=500-(int(y_goal)+250) #linear transformation followed by conversion to openCV corrdinate

		contours.append([[x_goal, y_goal, theta_goal]])

	contours=getMultipleContours(contours, 20)
	
	# extract x and y goals into 2 separate lists
	x_goals, y_goals, theta_goals=splitContours(contours)

	# print(x_goals, y_goals, theta_goals)
	return [x_goals, y_goals, theta_goals]

# generate goals wrt time
def getCurrGoal(curr_t):
	return 200*math.cos(curr_t), 100*math.sin(2*curr_t), (math.pi/4)*math.sin(curr_t)

# edits the array of contours-adds a third element to each pixel that acts as theta(0 for every waypoint) input
def addTheta(contours):
	for contour in contours:
		for i in range(len(contour)):
			contour[i][0].append(0)
			# print(contour[i][0])

	return contours

# returns a true value if any one pixel coordinate is closer than 10 pixels from the other
def isPxNearby(pixelA, pixelB, thresh):
	return abs(pixelA[0]-pixelB[0])<thresh and abs(pixelA[1]-pixelB[1])<thresh

def splitContours(contours):
	x_goals=[]
	y_goals=[]
	theta_goals=[]

	# split contours([[[x, y, theta]..,[]],[]...,[]]) to extract x_goals([[x1, x2, ...],[]...,[]), y_goals([[y1,y2...],[]...,[]]) and thata_goals([[theta1,theta2...],[]...,[]])
	for contour in contours:
		x_goal=[]
		y_goal=[]
		theta_goal=[]
		for i in range(len(contour)):
			# print(contour[i])
			x_goal.append(contour[i][0][0])
			y_goal.append(contour[i][0][1])
			theta_goal.append(contour[i][0][2])
		
		x_goals.append(x_goal)
		y_goals.append(y_goal)
		theta_goals.append(theta_goal)
	
	return x_goals, y_goals, theta_goals

def fillContours(contours, thresh):
	for contour in contours:
		i=0
		while i < len(contour):
			if i==len(contour)-1: break

			curr_px=contour[i][0]
			next_px=contour[i+1][0]

			if (abs(curr_px[0]-next_px[0])>thresh) or (abs(curr_px[1]-next_px[1])>thresh):
				new_x=int((curr_px[0]+next_px[0])/2)
				new_y=int((curr_px[1]+next_px[1])/2)
				new_px=np.array([[new_x, new_y]])
				
				# new_px=new_px.reshape(1,2)

				slobj = slice(0, i+1)
				# print(slobj)

				# print(new_px.shape, contour[0].shape)
				
				# contour[slobj] = new_px
				# # i-=1
				# print(contour[i+1])
				contour=np.insert(slobj, new_px)
				# i-=1
			i+=1

	return contours

# this function draws the contours then extracts all the drawn pixels to form a denser set of waypoints
def improveContours(contours):
	frame=np.ones([500, 500, 3])
	frame=cv2.drawContours(frame, contours, -1, (0, 0, 0), 1)
	contours.clear()

	width, height, _=frame.shape

	for w in range(width):
		for h in range(height):
			# print(frame[w][h])
			if(frame[h][w][0]==0): 
				# print(h, w)
				contours.append([[w, h, 0]]) #last 0 being appended is the theta goal

	return contours

# this function splits a series of waypoints into separate contours
def getMultipleContours(contours, thresh):
	contour=[]
	separated_contours=[]

	for  i in range(len(contours)):
		curr_px=contours[i][0]

		# if contour is empty or incoming pixel is close to the previous one, add them into the same contour
		if len(contour)==0:
			contour.append([curr_px])
		elif isPxNearby(contour[-1][0], curr_px, thresh):
			contour.append([curr_px])
		# else store the current contour, clear it and insert the current pixel
		else:
			separated_contours.append(contour)
			contour.clear()
			contour.append([curr_px])

	# append the last contour
	if(len(contour)!=0): separated_contours.append(contour)

	return separated_contours

