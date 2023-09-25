#!/usr/bin/env python3

'''
Team Id : HB1284
Author List : Sachin, Debrup
Filename: feed_utils.py
Theme: HoLA Bot
Functions: detect_aruco(), extract_arena_corners(), perspectiveTransform(), getPose(), resize(), getImage(), getContourMsg(), getContoursImg(), getContoursFunc()
		   getCurrGoal(), addTheta(), isPxNearby(), splitContours(), getMultipleContours(), diluteWaypoints()
Global Variables: None
'''

import cv2
import math
import rospy
import ast



'''
Function Name: verifyArgs
Input: arguments(manual input)
Output: loginfo , shutdown request 
Logic: 
	This function performs checking the arguments whether it is under arena size.
Example call: verifyArgs(parsed arguments)
'''
def verifyArgs(args):
	w, h = ast.literal_eval(args.frameSize)
	a, b = ast.literal_eval(args.frameStart)

	if w>500 or h>500:
		rospy.loginfo("Given Frame size exceeds arena size")
		rospy.signal_shutdown("Frame size exceeded")
	if (a+w > 500) or (b+h > 500):
		rospy.loginfo("frame will exceed the arena incase of given frame start coordinate")
		rospy.signal_shutdown("Not enough space!")

'''
Function Name: detect_aruco
Input: frame, aruco dictionary, aruco params
Output: aruco ids, aruco corners
Logic: 
	This function performs aruco marker detection on the imput frame and returns the list of detected aruco ids and their corresponding corners
Example call: id, corners=detect_aruco(frame, dict, params)
'''
def detect_aruco(aruco_frame, dict, params):
	#detect the markers in the frame
	corners, id, _ = cv2.aruco.detectMarkers(aruco_frame, dict, parameters=params)
	
	return id, corners

'''
Function Name: extract_arena_corners
Input: corner_msg(list of detected corners), id_msg(list of detected aruco ids), aruco_ids(list of aruco ids available at the corners of the arena)
Output: upper right, upper left, bottom right, bottom left corners' pixel coordinates
Logic: 
	This function takes in the corner and id lists and returns the pixel coordinates of the 4 corners of the arena.
Example call: uR, uL, bR, bL = extract_arena_corners(corners, ids, aruco_ids)
'''
def extract_arena_corners(corner_msg, id_msg, aruco_ids):
	upperLeft=None
	upperRight=None
	bottomLeft=None
	bottomRight=None
	
	for i in range(len(corner_msg)):
		# if the bottom right aruco marker is detected, store its bottom right corner 
		if(id_msg[i][0]==aruco_ids[0]): bottomRight=corner_msg[i][0][2]

		# if the bottom left aruco marker is detected, store its bottom left corner 
		if(id_msg[i][0]==aruco_ids[1]): bottomLeft=corner_msg[i][0][3]

		# if the upper right aruco marker is detected, store its upper right corner 
		if(id_msg[i][0]==aruco_ids[2]): upperRight=corner_msg[i][0][1]

		# if the upper left aruco marker is detected, store its upper left corner 
		if(id_msg[i][0]==aruco_ids[3]): upperLeft=corner_msg[i][0][0]
	
	return upperRight, upperLeft, bottomRight, bottomLeft

'''
Function Name: perspectiveTransform
Input: feed(image), arena_corners(list of 4 pixel coordinates), final_feed_corners(list of 4 corners of the output frame)
Output: transformed image
Logic: 
	This function takes in image, the list of coordinates it wants to perspective transform on and the list of pixel coordinates of the 4 corners of the final image.
	Then it performs the perspective transform followed by warpPerspective and finally returns the transformed frame.
Example call: feed = perspectiveTransform(image, target_corners, final_frame_corners)
'''
def perspectiveTransform(feed, arena_corners, final_feed_corners):
	mat=cv2.getPerspectiveTransform(arena_corners, final_feed_corners)
	feed=cv2.warpPerspective(feed, mat, (500, 500))

	return feed

'''
Function Name: getPose
Input: corners(corners of the aruco marker on the robot)
Output: x(x coordinate), y(y coordinate), theta(orientation)
Logic: 
	This function calculates the real time odometry of the robot.
Example call: x, y, th = getPose(corners)
'''
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

'''
Function Name: resize
Input: img(frame), w(output_width), h(output_height)
Output: resized frame
Logic: 
	This function takes in the image, target width and height and returns the resized frame
Example call: frame = resize(frame, w, h)
'''
def resize(img, w, h):
	return cv2.resize(img, (w, h))

'''
Function Name: getImage
Input: name(name of the image)
Output: cv2 frame
Logic: 
	This function takes in the image name (including the extension) and generates the cv2 frame using imread function
Example call: frame = getImage('smile.jpg)
'''
def getImage(name, frame_size):
	path='/home/kratos/cyborg_ws/src/eyrc_2022_hb/eyrc_hb_feed/src/taskImages/{}'.format(name)
	w, h = frame_size
	# read image
	image=cv2.imread(path)        
	# resize
	image=resize(image, w=w, h=h)

	return image

'''
Function Name: getContourMsg
Input: mode(function/Image mode), image(image that is to be drawn), density(resolution of the contours(used incase of image mode)),
	   points(number of points in the trajectory(used incase of function mode))
Output: contours(list of waypoints. Data Structure:[[[x1, x2, ..], [], ..., []]
													[[y1, y2, ..], [], ..., []]
													[[theta1, theta2, ..], [], ..., []]])
Logic: 
	This function takes in the mode, image and density(incase of image mode) and points(incase of function mode) and returns the waypoints that the robot has to traverse 
	inorder to draw the image/plot the function.
Example call: contours = getContoursMsg(mode=0, image=frame, density=6) ------> image mode
						 getContourMsg(mode=0, points=1000) ---------------> function mode
'''
def getContourMsg(mode=None, image=None, density=3, points=500, frame_size=(500, 500), frame_start=(0, 0)):
	if mode == 0:
		print("...Image mode selected...")
		contours=getContoursImg(image, density, frame_size, frame_start)
	if mode == 1:
		print("..Function mode selected...")
		contours=getContoursFunc(points, frame_size, frame_start)

	return contours

'''
Function Name: getContoursImg
Input: image(frame that's to be drawn), density(resolution of each contour) , frame_size(the size of frame) , frame_start(upper left corner)
Output: list containing goals along x, y axes and target orientation.
Logic: 
	This function takes in the image to be drawn alongwith the density of the contour. It then converts the image into a thresholded image(incase it isn't)
	this is followed by contouring and inclusion of theta goals in each pixel coordinate extracted. Meanwhile points are shifted according to the frame size and frame starting point,  The final set of contours are diluted depending on the value of
	'density'. Finally, a list containing goals along x, y axes and target orientation is returned.
Example call: contours = getContoursImg(frame, density)
'''
def getContoursImg(image, density, frame_size, frame_start):
	# Convert to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	# Threshold the image
	thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)[1]

	# Find the contours in the image
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
	
	waypoints=[]
	# Extract the contours who dont have a parent
	for i in range(len(contours)):
		# if i==0 or i==1 or i==2 or i==3 or i==4 or i==11: 
		waypoints.append(contours[i].tolist())

	# print(waypoints)

	# shifts image to the given frame
	waypoints = shiftImage(waypoints, frame_start)

	# add theta to each pixel to convert it to a waypoint
	waypoints=addTheta(waypoints)
	# dilye waypoint density
	waypoints=diluteWaypoints(waypoints, density)

	# extract x, y, theta goals into 3 separate lists
	x_goals, y_goals, theta_goals=splitContours(waypoints)
	
	return [x_goals, y_goals, theta_goals]

# shifts image to the frame within which it is supposed to be drawn
def shiftImage(contours, frame_start):
	a, b = frame_start

	for contour in contours:
		for i in range(len(contour)):
			contour[i][0][0]+=a
			contour[i][0][1]+=b
	
	return contours

'''
Function Name: getContoursFunc
Input: points(number of points the time range is going to be divided into) , frame_size , frame_start
Output: list containing goals along x, y axes and target orientation.
Logic: 
	This function takes in the number of points, gets goal for each time instance , transform the points to the local particular frame and appends it in a list to generate contour data
Example call: [x_g, y_g, theta_g] = getContoursFunc(points)
'''
def getContoursFunc(points, frame_size, frame_start):
	t=[0, 2*math.pi]
	a, b = frame_start
	w, h = frame_size

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

		x_goal=int(x_goal+a+(w/2)) #linear transformation
		y_goal=h-int(y_goal+b+(h/2)) #linear transformation followed by conversion to openCV corrdinate

		contours.append([[x_goal, y_goal, theta_goal]])

	contours=getMultipleContours(contours, 500)

	# extract x and y goals into 2 separate lists
	x_goals, y_goals, theta_goals=splitContours(contours)

	# print(x_goals, y_goals, theta_goals)
	return [x_goals, y_goals, theta_goals]

'''
Function Name: getCurrGoal
Input: curr_t(time value at a certain instant)
Output: x(t), y(t), theta(t) {parametric functions along x, y and theta axes respectively}.
Logic: 
	This function computes the odometry of the robot at time 'curr_t' according to the parametric equations(x(t), y(t), theta(t))
Example call: x, y, theta = getCurrGoal(t)
'''
def getCurrGoal(curr_t):
	x = 100*math.cos(curr_t)
	y = 100*math.sin(curr_t)
	theta = 0
		
	return x, y, theta

'''
Function Name: addTheta
Input: contours(contour data)
Output: contours(theta appended contours)
Logic: 
	This function takes in the contour data, iterates through it and for each pixel coordinate, appends a theta value of 0.
Example call: contours = addTheta(contours)
'''
def addTheta(contours):
	for contour in contours:
		for i in range(len(contour)):
			contour[i][0].append(0)

	return contours

'''
Function Name: isPxNearby
Input: pixelA(pixel coordinates of point A), pixelB(pixel coordinates of point B), thresh(threshold value-a gap more than this means pixel A and B are far from each other)
Output: boolean(near ot far)
Logic: 
	This function judges if 2 pixel coordinates are far from each other or not.
Example call: isFar = isPxNearby(a, b, thresh)
'''
def isPxNearby(pixelA, pixelB, thresh):
	return abs(pixelA[0]-pixelB[0])<thresh and abs(pixelA[1]-pixelB[1])<thresh

'''
Function Name: splitContours
Input: contours(contour data)
Output: x_goals([[x1, x2, ...], [], ..., []]), y_goals([[y1, y2, ...], [], ..., []]), theta_goals([[theta1, theta2, ...], [], ..., []])
Logic: 
	This function takes in the contour data and splits it to generate 3 lists.
Example call: x_g, y_g, th_g = splitContours(contours)
'''
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

'''
Function Name: getMultipleContours
Input: contours(contour data), thresh(pixel threshold)
Output: separated_contours(list of contours if points within the contour were too far away)
Logic: 
	This function basically groups close pixel coordinates together. 'close' here refers to whether 2 pixel coordinates 
	have a distance within 10px along both axes.
Example call: contours = getMultipleContours(contours, 10)
'''
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

'''
Function Name: diluteWaypoints
Input: waypoints(contour data), density(required resolution of the contour data)
Output: diluted_waypoints(contour data reduced to the required resolution)
Logic: 
	This function takes in the contour data and skips goals such that the desired density is obtained.
Example call: contours = diluteWaypoints(contours, density)
'''
def diluteWaypoints(waypoints, density):
	diluted_waypoints=[]

	# add a waypoint in every 'density' interval
	for contour in waypoints:
		diluted_contour=[]
		for i in range(len(contour)):
			if i==len(contour)-1 or i%density==0: diluted_contour.append(contour[i])

		diluted_waypoints.append(diluted_contour)

	return diluted_waypoints
