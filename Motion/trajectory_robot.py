#!/usr/bin/env python3
import time
import numpy as np
import math
import RPi.GPIO as GPIO

# --- Stacy's Modifications ---
import StacysArm as arm
import ultra as pos
# --- END Stacy's Modifications ---

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
	except:
		pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.1):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass


# --- Stacy's Modifications ---
def circle():
	# Stacy's Code to Circle 180 Degrees-ish
	t = .4
	move(speed_set, 'forward', 'left')
	time.sleep(2*t)
	motorStop()
	move(speed_set, 'backward', 'right')
	time.sleep(t)
	motorStop()
	move(speed_set, 'forward', 'left')
	time.sleep(2*t)
	motorStop()
	move(speed_set, 'backward', 'right')
	time.sleep(t)
	motorStop()
##	move(speed_set, 'forward', 'left')
##	time.sleep(2*t)
##	motorStop()
##	move(speed_set, 'backward', 'left')
##	time.sleep(t)
	motorStop()

def wiggle_back():
	t = .075
	move(speed_set, 'forward', 'left')
	time.sleep(t)
	move(speed_set, 'forward', 'right')
	time.sleep(t)
	motorStop()
	
def wiggle_fwd():
	t = .075
	move(speed_set, 'backward', 'left')
	time.sleep(t)
	move(speed_set, 'backward', 'right')
	time.sleep(t)
	motorStop()
	
def wiggle():
	dist = pos.checkdist()
	print(dist)
	if dist < 0.1:
		wiggle_back()
	else:
		wiggle_fwd()
		
def obst_avoid(i):
	range_keep = 0.1 # Avoidance distance
	dist = pos.checkdist()
	#print(dist)
	if dist > range_keep:
		return True
	else:
		print('Automatic obstacle avoidance mode')
		while dist < range_keep:
			#motorStop()
			#wiggle_back()
			dist = pos.checkdist()
		return False
		
# --- END Stacy's Modifications ---
        
def destroy():
	motorStop()
	GPIO.cleanup()             # Release resourceS


if __name__ == '__main__':
	rx=[30.0, 28.0, 28.0, 28.0, 26.0, 26.0, 24.0, 24.0, 22.0, 22.0, 22.0, 22.0, 22.0, 22.0, 20.0, 18.0, 16.0, 14.0, 12.0, 10.0]
	rx.reverse()
	ry=[30.0, 28.0, 26.0, 24.0, 22.0, 20.0, 18.0, 16.0, 14.0, 12.0, 10.0, 8.0, 6.0, 4.0, 2.0, 4.0, 6.0, 8.0, 10.0, 10.0]
	ry.reverse()
	cur_co=np.array([rx[0],ry[0]])   # current coordinate

	delta_s=[]    # comperative direction, a list of the moving vector
	for i in range(len(rx)):
		delta_s.append(np.array([rx[i],ry[i]])-cur_co )
		cur_co=np.array([rx[i],ry[i]])

	dot_product=[]
	angle=[]
	
	# --- Stacy's Modifications ---
	times=[]
	aquired = False
	# --- Stacy's Modifications END ---

	for i in range(1, len(delta_s)-1):
		dot= np.vdot(delta_s[i]/np.linalg.norm(delta_s[i]), delta_s[i+1]/np.linalg.norm(delta_s[i+1]))
		dot_product.append(dot)
		ang=math.acos(dot)/math.pi
		if dot<0:
			angle.append(-ang)
		else:
			angle.append(ang)

	for i in range(len(angle)):
		if 0<angle[i]<0.0001:
			angle[i]=0
			
	try:
		speed_set = 100     # rad/s
		r=0.1
		setup()
		
		# --- Stacy's Modifications ---
		arm.init()
		# --- Stacy's Modifications END ---
		
		for k in range(len(angle)):
			dist=np.linalg.norm(delta_s[k+1])
			t=dist/speed_set/r
			
			# --- Stacy's Modifications ---
			times.append(t)
			obst_avoid(i)
			# --- Stacy's Modifications END ---
			
			if angle[k] >0:   # turn right
							move(speed_set, 'backward', 'right')
							time.sleep(t/3)
			elif angle[k] ==0:
							move(speed_set, 'backward', 'no')
							time.sleep(t/2)
			else:
							move(speed_set, 'backward', 'left')
							time.sleep(t/2)
		motorStop()


		# --- Stacy's Modifications ---
		t1 = time.time()
		while not aquired:
			aquired = arm.runarm()
			if not aquired:
				res = time.time()-t1
				#print(res)
				if res > 18:
					time.sleep(1)
					t1 = time.time()
					wiggle()
		motorStop()
		circle()
		time.sleep(1)
		angle.reverse()
		times.reverse()
		i = 0
		for k in range(len(angle)):
			t=times[i]
			t = t/2
			i += 1
			obst_avoid(i)
			if angle[k] >0:   # turn right
					move(speed_set, 'backward', 'left')
					time.sleep(t)
			elif angle[k] ==0:
					move(speed_set, 'backward', 'no')
					time.sleep(t)
			else:
					move(speed_set, 'backward', 'right')
					time.sleep(t)
		motorStop()
		arm.drop()
		# --- END Stacy's Modifications ---
		
		
		destroy()
	except KeyboardInterrupt:
		destroy()

