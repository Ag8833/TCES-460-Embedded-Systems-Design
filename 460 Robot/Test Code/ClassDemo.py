from __future__ import print_function
from __future__ import division
from builtins import input
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
from multiprocessing import Process
import RPi.GPIO as GPIO
import time
import multiprocessing
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2


#------------------BRICKPI STUFF--------------------
BrickPiSetup()  # setup the serial port for communication

LS_port_number = PORT_1
US1_port_number = PORT_4	        # Define the port number here.
US2_port_number = PORT_2	# Define the port number here.  
US3_port_number = PORT_3	# Define the port number here.  

BrickPi.SensorType[US1_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US2_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US3_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[LS_port_number] = TYPE_SENSOR_LIGHT_ON   #Set the type of sensor 

BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor B

BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

#Communication timeout in ms (how long since last valid communication before floating the motors).
#0 disables the timeout so the motor would keep running even if it is not communicating with the RaspberryPi
BrickPi.Timeout=3000
#print("BrickPiSetTimeout Status :",BrickPiSetTimeout())

preDist = 0
distC = 0
distL = 0
distR = 0
speedL = 1
speedR = 1
#------------------BRICKPI STUFF---------------------

#------------------TURN FUNCTIONS--------------------
motor_offset = 2
motor_base_speed = 75
right_speed = 1
left_speed = -1


    

def motor_Stop():
    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()
    
def turn_right():
    BrickPi.MotorSpeed[PORT_A] = (-motor_base_speed + motor_offset)   #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = (motor_base_speed - motor_offset)   #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    while(time.time() - ot < .2):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motor_Stop()

def turn_left():
    BrickPi.MotorSpeed[PORT_A] = (motor_base_speed + motor_offset)  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = (-motor_base_speed - motor_offset)  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    while(time.time() - ot < .2):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motor_Stop()    

#------------------TURN FUNCTIONS--------------------


#------------------CANNON STUFF--------------------
cannonShots = 4
turretShots = 6

# Use GPIO numbers not pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin Definitons:
LLSPin = 23 # BCM pin Left Limit Switch 
RLSPin = 18 # BCM pin Right Limit Switch
SFPin  = 24 # BCM pin Shot Fired
ULSPin = 7  # BCM pin Upper Limit Switch
DLSPin = 25 # BCM pin Lower Limit Switch 


X1Pin = 22 # BCM pin x-axis Motor 1AY
X2Pin = 10 # BCM pin x-axis Motor 2AY
Y1Pin = 12  # BCM pin y-axis Motor 3AY
Y2Pin = 21 # BCM pin y-axis Motor 4AY
FCPin = 9  # BCM pin fire cannon
FTPin = 11 # BCM pin fire turret
LZRPin = 20 # BCM Laser

# set up the GPIO channels - 5 input and 5 output
GPIO.setup(LLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(RLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(SFPin,  GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(ULSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(DLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown

GPIO.setup(X1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(X2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FCPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FTPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(LZRPin, GPIO.OUT) # Sets Lazer on or off 

# Initial states for Motors:
GPIO.output(X1Pin, GPIO.LOW)
GPIO.output(X2Pin, GPIO.LOW)
GPIO.output(Y1Pin, GPIO.LOW)
GPIO.output(Y2Pin, GPIO.LOW)
GPIO.output(FCPin, GPIO.LOW)
GPIO.output(FTPin, GPIO.LOW)
GPIO.output(LZRPin, GPIO.LOW)

# Functions
left = GPIO.LOW
right = GPIO.LOW
up = GPIO.LOW
down = GPIO.LOW

def checkX():
    left = GPIO.input(LLSPin) # LeftBounds
    right = GPIO.input(RLSPin) # RightBounds
    
def checkY():
    up = GPIO.input(ULSPin) # LeftBounds
    down = GPIO.input(DLSPin) # RightBounds

def lookLeft():
    print ("Looking Left")
    GPIO.output(X1Pin, GPIO.LOW)
    GPIO.output(X2Pin, GPIO.HIGH)
				
def lookRight():
    print ("Looking Right")
    GPIO.output(X1Pin, GPIO.HIGH)
    GPIO.output(X2Pin, GPIO.LOW)
      				
def lookUp():
    print ("Looking Up")
    GPIO.output(Y1Pin, GPIO.LOW)
    GPIO.output(Y2Pin, GPIO.HIGH)
	
		
def lookDown():
    print ("Looking Down")
    GPIO.output(Y1Pin, GPIO.HIGH)
    GPIO.output(Y2Pin, GPIO.LOW)
		
def stopX():
    print ("stop X")
    GPIO.output(X1Pin, GPIO.LOW)
    GPIO.output(X2Pin, GPIO.LOW)

def stopY():
    print ("stop Y")
    GPIO.output(Y1Pin, GPIO.LOW)
    GPIO.output(Y2Pin, GPIO.LOW)		
def stopC():
    print ("stop C")
    GPIO.output(FCPin, GPIO.LOW)
               
def stopT():
    print ("stop T")
    GPIO.output(FTPin, GPIO.LOW)

def fireCannon():
    shotFired = GPIO.LOW
    laserOn()
    while ( shotFired == GPIO.LOW):
        GPIO.output(FCPin, GPIO.HIGH)
        time.sleep(.5)
        shotFired = GPIO.input(SFPin)
    GPIO.output(FCPin, GPIO.LOW)
    laserOff()
    time.sleep(.01)
    #cannonShots = (cannonShots - 1)
		
		
def fireTurret():
    GPIO.output(FTPin, GPIO.HIGH)
    time.sleep(.075)	
    GPIO.output(FTPin, GPIO.LOW)
    #turretShots = (turretShots - 1)
		
def cannonStop():
    GPIO.output(X1Pin, GPIO.LOW)
    GPIO.output(X2Pin, GPIO.LOW)
    GPIO.output(Y1Pin, GPIO.LOW)
    GPIO.output(Y2Pin, GPIO.LOW)
    GPIO.output(FCPin, GPIO.LOW)
    GPIO.output(FTPin, GPIO.LOW)
    laserOff()
    time.sleep(0.1)	

def laserOn():
    GPIO.output(LZRPin, GPIO.HIGH)

def laserOff():
    GPIO.output(LZRPin, GPIO.LOW)

    
def trackX(xOffset):
    print ("tracking X")
    checkX()
    if (xOffset > 0):
        if(right == GPIO.HIGH):
            turn_right()
        else:   
            lookRight()
            time.sleep(.01)
    else:
        if(left == GPIO.HIGH):
            turn_left()
        else:   
            lookLeft()
            time.sleep(.01)
    stopX()
    
def trackY(yOffset):
    print ("tracking Y")
    checkY()
    if (yOffset > 0):
        if(up == GPIO.HIGH):
            print ("upper limit")
        else:
            lookUp()
            time.sleep(.01)
    else:
        if(down == GPIO.HIGH):
            print ("lower limit")
        else:
            lookDown()
            time.sleep(.01)
    stopY()    
#------------------CANNON STUFF--------------------
 

#------------------OPENCV STUFF--------------------

def opencv(offset, event):
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.rotation = 180
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
 
    # allow the camera to warmup 
    time.sleep(0.1)

    face_cascade = cv2.CascadeClassifier('spherecascade40x40.xml')
    #eye_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_eye.xml')

    try:
        while(1):
            print ("in opencv")
            #rawCapture = PiRGBArray(camera, size=(640, 480))
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                faces = face_cascade.detectMultiScale(gray, 10, 10)
                #eyes = eye_cascade.detectMultiScale(gray, 1.1, 5)

                #print ("Found "+str(len(faces))+" face(s)")
                #print ("Found "+str(len(eyes))+" eyes(s)")

                for (x,y,w,h) in faces:
                    #print(x,y,w,h)
                    offset.value = faces
                    event.set()
                    cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                #for (x,y,w,h) in eyes:
                #        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
     
                # show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
     
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)

    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        print("Breaking")

## the main code for the cannon/turret/and laser

#------------------OPENCV STUFF--------------------        



#------------------Targeting STUFF--------------------        
def cannon(offset, event):
    #these are the centerpoints of the screen, if we change the resolution, we'll need to change these
    center_x = 100
    center_y = 75

    try:
        while(1):
            value = offset.value
            objects = int(len(value))
            for (x,y,w,h) in value:
                x_offset = -(center_x -(x -(w/2))) #this is to get the units in cartesian system
                y_offset = center_y -(y -(h/2))
                print(x_offset, y_offset)
                #print(objects)

                #if there is only one target
                if(objects is 1):   
                    #if the target is within 10 units of the center of the camera, fire
                    if(x_offset and y_offset < 10):
                        #print ("fire!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        fireCannon()
                    #otherwise track the target    
                    trackX(x_offset)
                    trackY(y_offset)                
                
        
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        cannonStop()
        GPIO.cleanup() # cleanup all GPIO

#------------------Targeting STUFF--------------------


#------------------Navigation STUFF--------------------         

def navigation():
    
    BrickPiUpdateValues()
    
    
    preDistC = 0
    preDistL = 0
    preDistR = 0
    speedL = 1
    speedR = 1
    
    while(1):
        BrickPiUpdateValues()
        distC = BrickPi.Sensor[US1_port_number]
        distL = BrickPi.Sensor[US2_port_number]
        distR = BrickPi.Sensor[US3_port_number]
        if(distL < 0 or distL > 255):
            distL =preDistL
        if(distC < 0 or distC > 255):
            distC =preDistC
        if(distR < 0 or distR > 255):
            distR =preDistR    
        print("Left - ", distL, ", Center - ", distC, ", Right - ", distR)
                
        #BEGIN -------------------- Driving Correction
        if(distC < 20):
            if((distL > 30) and (distR < 30)): #if left is more open than right
                while((distC < 30) and (distR < 30)):
                    turn_left()
                    BrickPiUpdateValues()
            else:
                while((distC < 30) and (distL < 30)):
                    turn_right()
                    BrickPiUpdateValues()
        if(distL < 20): #getting close to the left 
            speedR = distL/20 #slow the right wheel down porportionally 
        else:
            speedR = 1
        if(distR < 20): #getting close to the right side 
            speedL = distR/20 #slow the left wheel down porportionally 
        else:
            speedL = 1
            
                 
        BrickPi.MotorSpeed[PORT_A] = (motor_base_speed + motor_offset)*speedR
        BrickPi.MotorSpeed[PORT_D] = (motor_base_speed + motor_offset)*speedL
               
        preDistL = distL
        preDistR = distR
        preDistC = distC
        #END -------------------- Driving Correction

        
        #BEGIN - Checking Light Sensor
        if(BrickPi.Sensor[LS_port_number] < 550):
            turn_left()
            print("WHITE LINE - STOPPING")
        #END -------------------- Checking Light Sensor

                  
       
#------------------Navigation STUFF--------------------         

if __name__ == '__main__':
    mgr = multiprocessing.Manager()
    namespace = mgr.Namespace()
    event = multiprocessing.Event()
    
   
    s = multiprocessing.Process(target=opencv, args = (namespace, event))
    r = multiprocessing.Process(target=cannon, args = (namespace, event))
    q = Process(target=navigation)

    s.start()
    r.start()
    q.start()
    
    s.join()
    r.join()
    q.join()

