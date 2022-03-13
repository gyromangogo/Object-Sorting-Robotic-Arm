#Disclaimer: Check Read me

#import pyserial library for serial interfacing with python
#import opencv(cv2) library for image capture
#import numpy library for numerical and logical computation involving arrays

import serial
import cv2
import numpy as np
import time

#initialize serial port
#to check the port, go to Device Manager and Ports(COM & LPT)
#select the one where bluetooth is connected, in our case it was COM5
ser = serial.Serial('Serial Port', 9600, timeout=1) 

#initialize your camera for video capture, replace 1 with 0 while using default webcam
cap = cv2.VideoCapture(1)
#define frame width and frame height
frameWidth = 300
frameHeight = 180
cap.set(3, frameWidth)
cap.set(4, frameHeight)

#define function for color and size detection
def colordetect():
    _, frame = cap.read() #reads data from camera
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #converting RGB into HSV color space
    # Red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    redm = cv2.bitwise_and(frame, frame, mask=red_mask)
    # Blue color
    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    bluem = cv2.bitwise_and(frame, frame, mask=blue_mask)
    #for normal
    red = np.linalg.norm(redm) #taking normal value of HSV for red
    blue = np.linalg.norm(bluem) #taking normal value of HSV for blue
    #from normal value we can determine the size of the object according to the number of pixels 
    #for display
    cv2.imshow('Frame', frame)
    cv2.imshow('Red', redm)
    cv2.imshow('Blue', bluem)
    print("Red = ", red) #prints the normal value of red color
    print("Blue = ", blue) #prints the normal value of blue color
    if (red > 10000) or (blue > 10000): #numbers here represents the normal value 
        if (red <= 20000) and (red > blue):
            return "rs" #red small
        elif (red > 20000) and (red > blue):
            return "rb" #red big
        elif (blue <= 20000) and (red < blue):
            return "bs" #blue small
        elif (blue > 20000) and (red < blue):
            return "bb" #blue big
    elif (red < 9000) and (blue < 9000):
        return "null"

while True:
    user_input = colordetect()
    if user_input == "rs":
        #colordetect()
        ser.write(b'B') #send signal to arduino
        time.sleep(30)
    elif user_input == "rb":
        #colordetect()
        ser.write(b'D')
        time.sleep(30)
    elif user_input == "bs":
        #colordetect()
        ser.write(b'A')
        time.sleep(30)
    elif user_input == "bb":
        #colordetect()
        ser.write(b'C')
        time.sleep(30)
    key = cv2.waitKey(1)
    if key == 27:
        break
