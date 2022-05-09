import os
import cv2
from base_camera import BaseCamera
import numpy as np
import matplotlib.pyplot as plt
import time
import csv

colorUpper = np.array([255, 100, 100])
colorLower = np.array([175, 0, 0]) 
font = cv2.FONT_HERSHEY_SIMPLEX

class Camera(BaseCamera):  
	video_source = 0  
	def __init__(self):  
		if os.environ.get('OPENCV_CAMERA_SOURCE'):  
			Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))  
		super(Camera, self).__init__()
		  
	def set_video_source(source):  
		Camera.video_source = source   
	def frames():  
		aquired = False
		results_file = open('results.csv', mode='a')
		camera = cv2.VideoCapture(Camera.video_source)  
		if not camera.isOpened():  
			raise RuntimeError('Could not start camera.') 
		for i in range(0,50): 
		# read current frame  
			_, img = camera.read() #Obtain images captured by the camera  
			#print("Saving .jpg")
			cv2.imwrite("filename.jpg", img)
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #Transfrom the images to HSV color space   
			mask = cv2.inRange(hsv, colorLower, colorUpper) #Loop to detect the color based on the target color range in the HSV color space, and turn the color blocks into masks  
			mask = cv2.erode(mask, None, iterations=2)  #Erode and diminish the small masks (hot pixels) in the image (eliminate small color blocks or hot pixels)  
			mask = cv2.dilate(mask, None, iterations=2) #Dilate, to resize the large masks eroded in the previous line to the original  
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,  
			cv2.CHAIN_APPROX_SIMPLE)[-2]            #Find masks in the image  
			center = None         
			if len(cnts) > 0:   #If the number of masks is more than 1,  
				# Find the coordinate of the center and size of the target color object in the image
				c = max(cnts, key=cv2.contourArea)  
				((box_x, box_y), radius) = cv2.minEnclosingCircle(c)  
				M = cv2.moments(c)  
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
				X = int(box_x)  
				Y = int(box_y)  
				# Obtain and output the coordinate of the center of the target color object 
				print('X:%d'%X)  
				print('Y:%d'%Y)  
				print('-------')  
				# Show the text "Target Detected" in the image 
				cv2.putText(img,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA) 
				print('--------- Color object detected --------- ')
				results_file.write("Detected, " + str(X) + " " + str(Y) + "\n")
				# Testing Picture Saving
				#img = cv2.imread('filename.jpg')
				#filename = "./aquired/"
				#filename += (str(int(time.time())))
				#filename +=('.jpg')
				#cv2.imwrite(filename, img)
				aquired = True
				results_file.close()
				return 1
			else:  
				cv2.putText(img,'Target Detecting',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)  
				# Testing Picture Saving
				#img = cv2.imread('filename.jpg')
				#filename = "./not_aquired/"
				#filename += (str(int(time.time())))
				#filename +=('.jpg')
				#cv2.imwrite(filename, img)
		results_file.close()
		return 0