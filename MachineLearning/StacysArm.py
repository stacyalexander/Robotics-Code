#!/usr/bin/env python3
from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import ultra as pos
from camera import Camera
from LED import LED
import tensorflow as tf
from tensorflow.keras.models import load_model as lm
from skimage import io
from skimage.transform import resize as imresize
import numpy as np

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
led = LED

# Initial pwm positions - set in init function
pwm1_pos = 300;
pwm2_pos = 300;
pwm3_pos = 300;
pwm4_pos = 175;

def init():
    print("Initializing...")
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
    global mymodel
    mymodel = tf.keras.models.load_model('arm.h5')
    global sz
    sz = 64
    print("Ready")

def ml_verify():
    # predict_x[0] > 0.5 means aquired
    # predict_x[0] <= 0.5 means NOT aquired
    X_test = io.imread(str('filename.jpg'))
    X_test = (imresize(X_test, (sz, sz, 3)))
    X_test = np.expand_dims(X_test, axis=0)
    predict_x=mymodel.predict(X_test)
    match = predict_x[0][0]
    if match > 0.5:
        print("Bag")
        return True
    else:
        print("Not Bag")
        return False
        
def verify():
    aquired = False
    pwm.set_pwm(4,0,290)
    time.sleep(1)
    if (Camera.frames()):
        if(ml_verify()):
            print("Bag Aquired")
            aquired = True
            drop()
    if not aquired:
        pwm.set_pwm(4,0,320)
        #time.sleep(0.1)
        if (Camera.frames()):
            if(ml_verify()):
                print("Bag Aquired")
                aquired = True
                drop()
    if not aquired:
        pwm.set_pwm(4,0,350)
        #time.sleep(.1)
        if (Camera.frames()):
            if(ml_verify()):
                print("Bag Aquired")
                aquired = True
                drop()
    if not aquired:
            if(ml_verify()):
                print("Bag Aquired")
                aquired = True
                drop()
            else:
                print("Try To Grab Bag Again!")
    pwm.set_pwm(4,0,175)
    pwm.set_pwm(3,0,300)
    time.sleep(.1)
		
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
    pwm.set_pwm(4,0,175)
	

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
    for i in range(0,275):
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
            if pos.checkdist() < 0.11:
                pwm.set_pwm(4,0,175)
                led.colorWipe(0, 0, 255)  # blue
                if (Camera.frames()):
                    if (ml_verify()):
                        led.colorWipe(0, 255, 0)  # green
                        movearm()
                        led.colorWipe(0, 0, 0)
                elif (ml_verify()):
                    led.colorWipe(0, 255, 0)  # green
                    movearm()
                    led.colorWipe(0, 0, 0)
                else:
                    if (ml_verify()):
                        led.colorWipe(0, 255, 0)  # green
                        movearm()
                        led.colorWipe(0, 0, 0)
                    else:
                        print("Incorrect Color Detected")
                        led.colorWipe(255, 0, 0)  # red
                        time.sleep(.75)
                        led.colorWipe(0, 0, 0)
                        time.sleep(.75)
                        led.colorWipe(255, 0, 0)  # red
                        time.sleep(.75)
                        led.colorWipe(0, 0, 0)
    print("Exiting")
