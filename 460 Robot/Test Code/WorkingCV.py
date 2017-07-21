# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

print('IN OPENCV')
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (200, 200)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(200, 200))
 
# allow the camera to warmup 
time.sleep(0.1)

face_cascade = cv2.CascadeClassifier('sphere_cascade.xml')
#eye_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_eye.xml')

print('IN OPENCV')
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        print('IN OPENCV')
	image = frame.array
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

	faces = face_cascade.detectMultiScale(gray, 5, 5)
        #eyes = eye_cascade.detectMultiScale(gray, 1.1, 5)

        print "Found "+str(len(faces))+" face(s)"
        #print "Found "+str(len(eyes))+" eyes(s)"

        for (x,y,w,h) in faces:
                cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
        #for (x,y,w,h) in eyes:
        #        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
 
	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
