# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import math
import numpy as np

#dictionary of all contours
contours = {}
#array of edges of polygon
approx = []
#scale of the text
scale = .5

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (160, 120)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 120))
 
# allow the camera to warmup 
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(image,80,240,3)
        
    #contours
    canny2, contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for i in range(0,len(contours)):
        #approximate the contour with accuracy proportional to
        #the contour perimeter
        approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.02,True)

        #Skip small or non-convex objects
        if(abs(cv2.contourArea(contours[i]))<100 or not(cv2.isContourConvex(approx))):
                continue

        #triangle
        if(len(approx) == 3):
            x,y,w,h = cv2.boundingRect(contours[i])
            print("Found Triangle - ")
            print("x = ", x, ", y = ", y, ", w = ", w, ", h = ", h)
            cv2.putText(image,'TRI',(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,255),2,cv2.LINE_AA)
        else:
            #detect and label circle
            area = cv2.contourArea(contours[i])
            x,y,w,h = cv2.boundingRect(contours[i])
            radius = w/2
            if(abs(1 - (float(w)/h))<=2 and abs(1-(area/(math.pi*radius*radius)))<=0.2):
                print("Found Circle - ")
                print("x = ", x, ", y = ", y, ", w = ", w, ", h = ", h)
                cv2.putText(image,'CIRC',(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,255),2,cv2.LINE_AA)

    #Display the resulting frame
    cv2.imshow('frame',image)
    cv2.imshow('canny',canny)
    rawCapture.truncate(0)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
            break
