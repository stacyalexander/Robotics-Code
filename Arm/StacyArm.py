#!/usr/bin/env python3
from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import ultra as pos
from camera import Camera
from LED import LED

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
led = LED

# Initial pwm positions - set in init function
pwm1_pos = 300;
pwm2_pos = 300;
pwm3_pos = 300;
pwm4_pos = 175;

def init():
	# Set arm to center postion
	pwm.set_pwm(0,0,400)
	# Set arm to up postion
	pwm.set_pwm(1,0,300)
	# Set hand to up (out) postition
	pwm.set_pwm(2,0,300)
	# Open Gripper
	pwm.set_pwm(3,0,300)
	# Camera Down
	pwm.set_pwm(4,0,175)
	# Turn off LEDs
	led.colorWipe(0, 0, 0)
	
def verify():
	aquired = False
	pwm.set_pwm(4,0,290)
	time.sleep(2)
	if (Camera.frames()):
		print("Bag Aquired")
		aquired = True
		drop()
	if not aquired:
		pwm.set_pwm(4,0,320)
		time.sleep(0.1)
		if (Camera.frames()):
			print("Bag Aquired")
			aquired = True
			drop()
	if not aquired:
		pwm.set_pwm(4,0,350)
		time.sleep(.1)
		if (Camera.frames()):
			print("Bag Aquired")
			aquired = True
			drop()
	if not aquired:
		print("Try To Grab Bag Again!")
	pwm.set_pwm(4,0,175)
	pwm.set_pwm(3,0,300)
	time.sleep(1)
		
def drop():
	global pwm1_pos
	global pwm2_pos
	global pwm3_pos
	# Open Gripper
	for i in range(0,250):
		pwm.set_pwm(3,0,(50+i))
		pwm3_pos += 1;
		time.sleep(0.005)
	# Hand down
	for i in range(0,150):
		pwm.set_pwm(2,0,(420-i))
		pwm2_pos -= 1;
		time.sleep(0.005)
	# Arm Down 
	pwm.set_pwm(1,0,200)
	time.sleep(1)
	pwm.set_pwm(1,0,300)
	

def movearm():
	global pwm1_pos
	global pwm2_pos
	global pwm3_pos
	global completed
	# Arm Down
	for i in range(0,50):
		pwm.set_pwm(1,0,(300-i))
		pwm1_pos -= 1;
		time.sleep(0.005)
	# Hand out and Arm Down
	for i in range(0,130):
		pwm.set_pwm(2,0,(300+i))
		pwm.set_pwm(1,0,(250-i))
		pwm1_pos -= 1;
		pwm2_pos += 1;
		time.sleep(0.005)
	# Close Gripper
	for i in range(0,250):
		pwm.set_pwm(3,0,(300-i))
		pwm3_pos -= 1;
		time.sleep(0.005)
	# Arm Up	
	for i in range(0,225):
		pwm.set_pwm(1,0,(150+i))
		pwm1_pos += 1;
		time.sleep(0.005)
	verify()

if __name__ == '__main__':
	led = LED()
	init()
	while 1:
		if pos.checkdist() < 0.11:
			led.colorWipe(0, 0, 255)  # blue
			results_file = open('results.csv', mode='a')
			if (Camera.frames()):
			#if pos.checkdist() > 0.08:
				led.colorWipe(0, 255, 0)  # green
				results_file.write("US Sensor," + str(pos.checkdist()) + "\n")
				results_file.close()
				movearm()
			else:
				print("Incorrect Color Detected")
				led.colorWipe(255, 0, 0)  # red
				time.sleep(.75)
				led.colorWipe(0, 0, 0)
				time.sleep(.75)
				led.colorWipe(255, 0, 0)  # red
				time.sleep(.75)
			led.colorWipe(0, 0, 0)
		#print(pos.checkdist())
		time.sleep(0.05)
	print("Exiting")
