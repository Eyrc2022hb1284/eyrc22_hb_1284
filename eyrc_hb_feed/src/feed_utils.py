import cv2
import math

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

# constructs the path and generates the image
def getImage(name):
	path='/home/kratos/cyborg_ws/src/eyrc_2022_hb/eyrc_hb_feed/src/taskImages/{}'.format(name)
	# read image
	image=cv2.imread(path)        
	# resize
	image=resize(image, w=500, h=500)

	return image

# extract contour coordinates from image/function according to the chosen mode
def getContourMsg(mode=None, image=None, density=3, points=500):
	if mode == 0:
		print("...Image mode selected...")
		contours=getContoursImg(image, density)
	if mode == 1:
		print("..Function mode selected...")
		contours=getContoursFunc(points)

	return contours

# contour extraction algo
def getContoursImg(image, density):
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
	# dilye waypoint density
	waypoints=diluteWaypoints(waypoints, density)

	# extract x, y, theta goals into 3 separate lists
	x_goals, y_goals, theta_goals=splitContours(waypoints)
	
	return [x_goals, y_goals, theta_goals]

# function plotting algo
def getContoursFunc(points):
	t=[0, 2*math.pi]

	# lower limit of time
	t_low=t[0]
	# upper limit of time
	t_high=t[1]
	# store waypoints
	contours=[]

	for i in range(points):
		curr_t=t_low+i*(t_high-t_low)/points

		# get current pixel
		x_goal, y_goal, theta_goal=getCurrGoal(curr_t)

		x_goal=int(x_goal)+250 #linear transformation
		y_goal=500-(int(y_goal)+250) #linear transformation followed by conversion to openCV corrdinate

		contours.append([[x_goal, y_goal, theta_goal]])

	# split into multiple waypoints if they are too distant
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

	return contours

# returns a true value if any one pixel coordinate is closer than 'thresh' pixels from the other
def isPxNearby(pixelA, pixelB, thresh):
	return abs(pixelA[0]-pixelB[0])<thresh and abs(pixelA[1]-pixelB[1])<thresh

# splits [[[x, y, theta]], [[]], ....] into separate x, y, theta goals
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
			x_goal.append(contour[i][0][0])
			y_goal.append(contour[i][0][1])
			theta_goal.append(contour[i][0][2])
		
		x_goals.append(x_goal)
		y_goals.append(y_goal)
		theta_goals.append(theta_goal)
	
	return x_goals, y_goals, theta_goals

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

# reduce density of the waypoints to 'density' no.of pixels between consecutive waypoints
def diluteWaypoints(waypoints, density):
	diluted_waypoints=[]

	# add a waypoint in every 'density' interval
	for contour in waypoints:
		diluted_contour=[]
		for i in range(len(contour)):
			if i==len(contour)-1 or i%density==0: diluted_contour.append(contour[i])

		diluted_waypoints.append(diluted_contour)

	return diluted_waypoints