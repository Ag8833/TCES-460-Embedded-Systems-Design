#Code for autonomous robot to navigate, recognize drones/cars, and track/shoot at them
#Created by: Reagan Stovall and Andrew Gates

from __future__ import print_function
from __future__ import division
from builtins import input

from multiprocessing import Process, Pipe

import time
import multiprocessing
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

#------------------GPIO Setup--------------------
import RPi.GPIO as GPIO

#Use GPIO numbers not pin numbers

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Pin Definitons:
LLSPin = 23 # BCM pin Left Limit Switch 
RLSPin = 18 # BCM pin Right Limit Switch
SFPin  = 24 # BCM pin Shot Fired
ULSPin = 7  # BCM pin Upper Limit Switch
DLSPin = 25 # BCM pin Lower Limit Switch 

X1Pin = 22 # BCM pin x-axis Motor 1AY
X2Pin = 10 # BCM pin x-axis Motor 2AY
Y1Pin = 12  # BCM pin y-axis Motor 3AY
Y2Pin = 21 # BCM pin y-axis Motor 4AY
FCPin = 16  # BCM pin fire cannon
FTPin1 = 13 # BCM pin fire turret
FTPin2 = 26 # BCM pin fire turret
LZRPin = 20 # BCM Laser

ButtonPin = 4 # BCM pin for reload
leftLineSensor = 6 # BCM pin for the left line sensor
rightLineSensor = 5 # BCM pin for the right line sensor

#Set up the GPIO channels - 5 input and 5 output
GPIO.setup(LLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(RLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(SFPin,  GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(ULSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(DLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(ButtonPin, GPIO.IN, GPIO.PUD_UP)

GPIO.setup(X1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(X2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FCPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FTPin1,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FTPin2,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(LZRPin, GPIO.OUT) # Sets Lazer on or off 

#Initial states for Motors:
GPIO.output(X1Pin, GPIO.LOW)
GPIO.output(X2Pin, GPIO.LOW)
GPIO.output(Y1Pin, GPIO.LOW)
GPIO.output(Y2Pin, GPIO.LOW)
GPIO.output(FCPin, GPIO.LOW)
GPIO.output(FTPin1, GPIO.LOW)
GPIO.output(FTPin2, GPIO.LOW)
GPIO.output(LZRPin, GPIO.LOW)
#------------------GPIO Setup--------------------

#--------------------MPU Setup--------------------
import FaBo9Axis_MPU9250
import sys

mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#--------------------MPU Setup--------------------

#------------------BRICKPI Setup--------------------
from BrickPi import * #import BrickPi.py file to use BrickPi operations

BrickPiSetup()  # setup the serial port for communication

US1_port_number = PORT_4	# Define the port number here.
US2_port_number = PORT_3	# Define the port number here.  
US3_port_number = PORT_2	# Define the port number here.  

BrickPi.SensorType[US1_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US2_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US3_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1

BrickPi.MotorEnable[PORT_B] = 1 #Enable the Motor B
BrickPi.MotorEnable[PORT_C] = 1 #Enable the Motor C

BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

#Communication timeout in ms (how long since last valid communication before floating the motors).
#0 disables the timeout so the motor would keep running even if it is not communicating with the RaspberryPi
BrickPi.Timeout=3000
#print("BrickPiSetTimeout Status :",BrickPiSetTimeout())

#------------------BRICKPI Setup--------------------

#------------------Cannon Setup--------------------
left = GPIO.input(LLSPin)  # LeftBounds
right = GPIO.input(RLSPin) # RightBounds
up = GPIO.input(ULSPin)    # LeftBounds
down = GPIO.input(DLSPin)  # RightBounds

#------------------Cannon Setup--------------------

#------------------TURN FUNCTIONS--------------------
def turn_180(right = 1):
    BrickPi.MotorSpeed[PORT_B] = 100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = -100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    while(time.time() - ot < 1.7):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()

def turn_90(right = 1):
    BrickPi.MotorSpeed[PORT_B] = 100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = -100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .75   # Half of 180
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()

def turn_45(right = 1):
    BrickPi.MotorSpeed[PORT_B] = 75 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = -75 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = 1.5/2 + 0.11  # Half of 90
    while(time.time() - ot < wait): #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()

def turn_right(): #generically turn right
    BrickPi.MotorSpeed[PORT_B] = 75  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = -75 #Set the speed of MotorB (-255 to 255)
    BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
    time.sleep(.4)
    motorStop()
    
def turn_left(): #generically turn left
    BrickPi.MotorSpeed[PORT_B] = -75  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = 75 #Set the speed of MotorB (-255 to 255)
    BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
    time.sleep(.4)
    motorStop()      

def bounceRight_45():
    BrickPi.MotorSpeed[PORT_B] = 0  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = 100   #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .72/4 + 0.11  # Half of 90
    while(time.time() - ot < wait): #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()

def bounceLeft_45():
    BrickPi.MotorSpeed[PORT_B] = 100 #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_C] = 0   #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .72/4 + 0.11  # Half of 90
    while(time.time() - ot < wait):  #running while loop for 3 seconds
        BrickPiUpdateValues()        # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()      

def reverse():
    BrickPi.MotorSpeed[PORT_B] = -75
    BrickPi.MotorSpeed[PORT_C] = -75
    ot = time.time()
    wait = 1  # Half of 90
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.02)
    motorStop()
    
def motorStop(): # Stop moving
    BrickPi.MotorSpeed[PORT_B] = 0
    BrickPi.MotorSpeed[PORT_C] = 0
    BrickPiUpdateValues()
    
#------------------TURN FUNCTIONS--------------------
    
#------------------Line FUNCTIONS--------------------

#sends an ir signal then reads the bounce
def readLeftLineSensor():
    GPIO.setup(leftLineSensor, GPIO.OUT)
    GPIO.output(leftLineSensor, GPIO.HIGH)
    time.sleep(.00001)#sleep for 10 microseconds
    GPIO.setup(leftLineSensor, GPIO.IN)
    leftLS = GPIO.input(leftLineSensor)
    thisTime = time.time()
    #measures the bounce time, if greater the 30ms, don't worry about it
    while((leftLS == GPIO.HIGH)and((time.time()-thisTime)<(.003))):
        leftLS = GPIO.input(leftLineSensor)
    diff = time.time()-thisTime

    return diff

def readRightLineSensor():
    GPIO.setup(rightLineSensor, GPIO.OUT)
    GPIO.output(rightLineSensor, GPIO.HIGH)
    time.sleep(.00001)#sleep for 10 microseconds
    GPIO.setup(rightLineSensor, GPIO.IN)
    rightLS = GPIO.input(rightLineSensor)
    thisTime = time.time()
    #measures the bounce time, if greater the 30ms, don't worry about it
    while((rightLS == GPIO.HIGH)and((time.time()-thisTime)<(.003))):
        rightLS = GPIO.input(rightLineSensor)
    diff = time.time()-thisTime

    return diff

#------------------CANNON Functions--------------------

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
    GPIO.output([FTPin1,FTPin2], GPIO.LOW)

def fireCannon(conn3):
    while(1):
        if(conn3.poll(.5) == 1):
            target = conn3.recv()
        else:
            target = 0
        if(target == 1):
            shotFired = GPIO.LOW
            laserOn()
            GPIO.output(FCPin, GPIO.HIGH) #fire on
            time.sleep(.75)
            while (shotFired == GPIO.LOW):
                shotFired = GPIO.input(SFPin)
                    
            GPIO.output(FCPin, GPIO.LOW) #fire off
            laserOff()
            time.sleep(.01)
        		
def fireTurret1():
    laserOn()
    GPIO.output(FTPin1, GPIO.HIGH)
    time.sleep(.055)	
    GPIO.output(FTPin1, GPIO.LOW)
    laserOff()

def fireTurret2():
    laserOn()
    GPIO.output(FTPin2, GPIO.HIGH)
    time.sleep(.0375)	
    GPIO.output(FTPin2, GPIO.LOW)
    laserOff()
		
def cannonStop():
    GPIO.output(X1Pin, GPIO.LOW)
    GPIO.output(X2Pin, GPIO.LOW)
    GPIO.output(Y1Pin, GPIO.LOW)
    GPIO.output(Y2Pin, GPIO.LOW)
    GPIO.output(FCPin, GPIO.LOW)
    GPIO.output([FTPin1,FTPin2], GPIO.LOW)
    laserOff()
    time.sleep(0.1)	

def laserOn():
    GPIO.output(LZRPin, GPIO.HIGH)

def laserOff():
    GPIO.output(LZRPin, GPIO.LOW)

def trackX(xOffset):
    print ("tracking X")
    trackSpeedX = abs(xOffset)/2000
    checkX()
    if (xOffset > 0):
        if(right == GPIO.HIGH):
            print ("right limit")
        else:   
            lookRight()
            time.sleep(trackSpeedX)
    else:
        if(left == GPIO.HIGH):
            print ("left limit")
        else:   
            lookLeft()
            time.sleep(trackSpeedX)
    stopX()
    
def trackY(yOffset):
    print ("tracking Y")
    trackSpeedY = abs(yOffset)/2000
    checkY()
    if (yOffset > 0):
        if(up == GPIO.HIGH):
            print ("upper limit")
        else:
            lookUp()
            time.sleep(trackSpeedY)
    else:
        if(down == GPIO.HIGH):
            print ("lower limit")
        else:
            lookDown()
            time.sleep(trackSpeedY)
    stopY()
#------------------CANNON--------------------

#------------------OPENCV--------------------
def opencv(conn, conn2, conn4, conn5):
    #Initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (208, 208)
    camera.rotation = 180
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(208, 208))
    time.sleep(0.1)

    #Setup variables for detection and ammunition
    droneFound = False
    carFound = False
    reloadMode = False
    defenseMode = True
    centered = False
    cannonShots = 4
    turretShots = 12
    shotFired = 0

    #Setup cascades for drones, cars and reloads
    drone_cascade = cv2.CascadeClassifier('sphere_cascade.xml')
    car_cone_cascade = cv2.CascadeClassifier('20_stage_normal_cone.xml')
    reload_cone_cascade = cv2.CascadeClassifier('stage_20_upsidedown_cone.xml')

    try:
        while(1):
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                #print('IN OPENCV')
                #Capture the image from the frame, and convert to gray
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                
                if(defenseMode == False):
                    #image = cv2.flip(image,0)
                    reloads = reload_cone_cascade.detectMultiScale(gray, 1.05, 1)
                else:
                    drones = drone_cascade.detectMultiScale(gray, 1.3, 3)#1.2,2
                    cars = car_cone_cascade.detectMultiScale(gray, 1.2, 2)

                #Scan the image for drones, cars and reloads
                #drones = drone_cascade.detectMultiScale(gray, 1.3, 3)#1.2,2
                #cars = car_cone_cascade.detectMultiScale(gray, 1.2, 2)
                #reloads = reload_cone_cascade.detectMultiScale(gray, 1.05, 1)

                #BEGIN -------------------- Defense Mode
                if(defenseMode == True):
                    #Check to see if either drones or cars is empty
                    if(int(len(drones)) == 0):
                       droneFound = False
                    if(int(len(cars)) == 0):
                       carFound = False

                    #If a drone is found, there is cannon ammunition, and no car is found
                    if(carFound == False and cannonShots > 0):
                        for (x,y,w,h) in drones:
                            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
                            roi_gray = gray[y:y+h, x:x+w]
                            roi_color = image[y:y+h, x:x+w]
                       
                            conn.send(1)
                            #conn5.send(1)
                            conn2.send(2)
                            conn.send(drones)
                            droneFound = True
                            carFound = False

                     #If a car is found, there is turret ammunition, and no drone is found
                    if(droneFound == False and turretShots > 0):
                        for (x,y,w,h) in cars:
                            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                            roi_gray = gray[y:y+h, x:x+w]
                            roi_color = image[y:y+h, x:x+w]
                       
                            conn.send(2)
                            #conn5.send(1)
                            conn2.send(2)
                            conn.send(cars)
                            carFound = True
                            droneFound = False

                    if(droneFound == False and carFound == False):
                        conn.send(0)
                        conn5.send(0)
                        #conn2.send(2)
                #END -------------------- Defense Mode

                #BEGIN -------------------- Reload Mode
                else:
                    #conn5.send(0)
                    x_offset = 0 #this is for navigating towards the cone.
                    center = 50  #the center of the screen
                    if(not centered):
                        find_start_X()
                        find_start_Y()
                        centered = True
                    for (x,y,w,h) in drones:
                        cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
                        roi_gray = gray[y:y+h, x:x+w]
                        roi_color = image[y:y+h, x:x+w]
                        x_offset = ((center -(x -(w/2)))) #find the offset of the cone          
                    #print("xoffset ", x_offset)
                    conn4.send(x_offset)    
                    conn.send(0)
                #END -------------------- Reload Mode

                #Show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
                #Clear the stream in preparation for the next frame
                rawCapture.truncate(0)
                
                #BEGIN -------------------- Ammunition 
                shotFired = conn.recv()
                if(shotFired == 1):
                    print("Cannon Shot Fired")
                    cannonShots -= 1
                    print("Cannon Shots Left - ", cannonShots)
                    if(cannonShots < 1):
                        droneFound = False
                elif(shotFired == 2):
                    print("Turret Shot Fired")
                    turretShots -= 1
                    print("Turret Shots Left - ", turretShots)
                    if(turretShots < 1):
                        carFound = False
                #END -------------------- Ammunition 

                #BEGIN -------------------- Mode Selection
                #If no ammunition left, switch to reload mode
                if(cannonShots < 1 and turretShots < 1):
                    defenseMode = False
                    reloadMode = True
                    print("IN RELOAD MODE")
                    conn2.send(1)

                    #If button is pressed, switch to defense mode
                    buttonResult = GPIO.input(ButtonPin)
                    if(GPIO.input(ButtonPin) == 1):
                        print("SWITCHING TO DEFENSE MODE")
                        defenseMode = True
                        reloadMode = False
                        centered = False
                        cannonShots = 4
                        turretShots = 12
                        conn2.send(0)
                        conn.send(3)

                #If ammunition is left, maintain defense mode
                elif(defenseMode == True):
                    conn2.send(0)
                #END -------------------- Mode Selection
                
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        conn.close()
        conn2.close()
        
        print("Breaking in OpenCV")
#------------------OPENCV--------------------

#------------------CANNON--------------------        
def cannon(conn, conn3):
    GPIO.setmode(GPIO.BCM)
    #Centerpoints of the screen
    center_x = 50
    center_y = 50
    #Cannon/Turret ammunition
    cannonShots = 4
    turretShots = 12
    startTime = time.time()
    whyTime = time.time()

    try:
        while(1):
            conn.send(0)
            target = conn.recv()
            #target = 0: Inactive 
##            if(target == 0):
##                #print("Cannon Inactive")
            if(time.time() > whyTime + 10):
                find_start_Y()
                conn.send(0)
                whyTime = time.time()
##                find_start_X()
##                conn.send(0)
##                    
##                    whyTime = time.time()
            #target = 3: Restarting Defense Mode
            if(target == 3):
                cannonShots = 4
                turretShots = 12
                whyTime = time.time()
            #BEGIN -------------------- Drone Tracking
            if(target == 1):
                #Receive the array of coordinates for drones found
                value = conn.recv()
                objects = int(len(value))
                print("Drones - ", objects)
                print(value)
                for (x,y,w,h) in value:
                    #this is to get the units in cartesian system
                    x_offset = -((center_x -(x -(w/2))))
                    y_offset = (center_y -(y -(h/2)))
                    print(x_offset, y_offset)

                    #If there is only one target
                    if(objects is 1):   
                        #If the target is within 10 units of the center of the camera, fire
                        if(abs(x_offset) < 15 and abs(y_offset < 15) and cannonShots > 0 and time.time() > startTime):
                            print ("CANNON fire!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            conn.send(1)
                            conn3.send(1)
                            startTime = time.time() + 2
                            cannonShots -= 1
                            
                        #Otherwise track the target    
                        trackX(x_offset)
                        trackY(y_offset)
                whyTime = time.time()
            #END -------------------- Drone Tracking

            #BEGIN -------------------- Car Tracking 
            if(target == 2):
                #Receive the array of coordinates for cars found
                value = conn.recv()
                objects = int(len(value))
                print ("Cars - ", objects)
                print(value)
                for (x,y,w,h) in value:
                    #This is to get the units in cartesian system
                    x_offset = -((center_x -(x -(w/2))))
                    y_offset = (center_y -(y -(h/2)))
                    print(x_offset, y_offset)

                    #If there is only one target
                    if(objects is 1):   
                        #If the target is within 10 units of the center of the camera, fire
                        if(abs(x_offset) < 15 and abs(y_offset < 15)and turretShots > 0 and time.time() > startTime):
                            print ("TURRET fire!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            conn.send(2)
                            startTime = time.time() + .5
                            if(turretShots > 6):
                                fireTurret1()
                            else:
                                fireTurret2()
                            turretShots -= 1
                            
                        #Otherwise track the target    
                        trackX(x_offset)
                        trackY(y_offset)
                whyTime = time.time()           
             #END -------------------- Drone Tracking
        
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        conn.close()
        cannonStop()
        GPIO.cleanup() # cleanup all GPIO
#------------------CANNON--------------------

        
#------------------STARTUP--------------------
   
def startup():
    
    find_start_Y()
    find_start_X()
    
#Function to set the X position of Cannon to neutral
def find_start_X():
    try:
        left = GPIO.input(LLSPin) # LeftBounds
        right = GPIO.input(RLSPin) # RightBounds
        while (left == GPIO.LOW):
            
            lookLeft()
            time.sleep(.01)
            stopX()
            left = GPIO.input(LLSPin) # LeftBounds
            
        startx_time = time.time()
        
        while (right == GPIO.LOW):
            
            lookRight()
            time.sleep(.01)
            stopX()
            right = GPIO.input(RLSPin) # RightBounds
            
        stopx_time = time.time()
        print(stopx_time)
        mid_time = (stopx_time - startx_time)/2
        while (time.time() < (stopx_time + mid_time)):
            print(left)
            lookLeft()
            time.sleep(.01)
            stopX()
            left = GPIO.input(LLSPin) # LeftBounds
        return mid_time    
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        stopX()
        GPIO.cleanup()

#Function to set the Y position of Cannon to neutral
def find_start_Y():
    try:
        up = GPIO.input(ULSPin) # LeftBounds
        down = GPIO.input(DLSPin) # RightBounds
        while (up == GPIO.LOW):
            
            lookUp()
            time.sleep(.01)
            stopY()
            up = GPIO.input(ULSPin) # upperBounds
         
        lookDown()
        time.sleep(.1)
        stopY()
                    
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        stopX()
        GPIO.cleanup()   
#------------------STARTUP--------------------

#------------------NAVIGATION--------------------
def navigation(conn2, conn4, conn5):
    preDist = 0
    dist = 0
    distL = 0
    distR = 0
    preDistL = 0
    preDistR = 0
    speedL = 1
    speedR = 1
    lean = 0
    reloadFlag = False
    targetFound = False
    startTime = time.time()
    mode = 0
    prevMode = 0
    flag = False
    
    prevTargetFound = False
    timer = time.time() + 10
    
    while(1):
        if(conn2.poll(.01) == 1):
            mode = conn2.recv()
        else:
            mode = prevMode
        
        
        print("mode: ", mode)
        if(mode == 0):
            #print("In navigation - DEFENSE MODE")
            lean = 0
            targetFound = False
        elif(mode == 1): #mode = reload 
            #print("In navigation - RELOAD MODE")
            #BrickPi.MotorSpeed[PORT_B] = 0  #Set the speed of MotorA (-255 to 255)
            #BrickPi.MotorSpeed[PORT_C] = 0
            targetFound = False
            
            if(conn4.poll(.5) == 1):
                lean = conn4.recv()
            else:
                lean = 0
            print("lean: " , lean)
        elif(mode == 2):
            targetFound = True
            #time.sleep(.25)
            
        BrickPiUpdateValues()
        dist = BrickPi.Sensor[US1_port_number]
        distL = BrickPi.Sensor[US2_port_number]
        distR = BrickPi.Sensor[US3_port_number]
        if(dist == -1):
            dist = 255
        if(distL == -1):
            distL = 255
        if(distR == -1):
            distR = 255
        print("Left - ", distL,", Front - ", dist,  ", Right - ", distR)
                    
            #BEGIN -------------------- Driving Correction
        if(targetFound is False and prevTargetFound is False):
            if(distL < 10):
                reverse()
                turn_45(-1)
            if(distR < 10):
                reverse()
                turn_45(1)
            #if(scanFlag is False):
            if(abs(distL - distR) < 10):
                BrickPi.MotorSpeed[PORT_B] = 75 + int(lean/2) #Set the speed of MotorA (-255 to 255)
                BrickPi.MotorSpeed[PORT_C] = 76 - int(lean/2)
                BrickPiUpdateValues()
                print(lean)
            if(distL > 50 and distR > 50):
                BrickPi.MotorSpeed[PORT_B] = 75 + int(lean/2) #Set the speed of MotorA (-255 to 255)
                BrickPi.MotorSpeed[PORT_C] = 76 - int(lean/2)
                BrickPiUpdateValues()
                
                #print(lean)
                
            elif(distL < distR):
                if(speedR > 0):
                    speedR = 0
                if(speedR <= -70):
                    speedR = -70
                else:
                    speedR = speedR - 8 

                speedL = speedL + 2

                if(speedL > 1):
                    speedL = 1
                            
                BrickPi.MotorSpeed[PORT_B] = 75 + speedR
                BrickPi.MotorSpeed[PORT_C] = 76
                BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
                time.sleep(.02)
            else:
                if(speedL > 0):
                    speedL = 0
                if(speedL <= -70):
                    speedL = -70
                else:
                    speedL = speedL - 8 
                speedR = speedR + 2 
               
                if(speedR > 1):
                    speedR = 1
                
                BrickPi.MotorSpeed[PORT_B] = 75 
                BrickPi.MotorSpeed[PORT_C] = 76 + speedL
                BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
                time.sleep(.02)

                #print("speedL = ", speedL, ", speedR = ", speedR)
                preDistL = distL
                preDistR = distR
        #end TargetFlag
        elif(targetFound is True or prevTargetFound is True):
            BrickPi.MotorSpeed[PORT_B] = 0 
            BrickPi.MotorSpeed[PORT_C] = 0
            BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
            
            time.sleep(2)
            print("Target Found")
            #END -------------------- Driving Correction
        
        #BEGIN  -------------------- Checking Line Sensor
        
        #line sensors
        leftLine = True
        rightLine = True
        #.003 will read positive on black,
        #.001 false on white
        lightOffset = .00011

        if(targetFound is False and prevTargetFound is False):
            while( leftLine is True or rightLine is True):
                if(conn4.poll(.001) == 1):
                    lean = conn4.recv()
                leftSide = readLeftLineSensor()
                rightSide = readRightLineSensor()
                print(leftSide, rightSide)
                if(leftSide < lightOffset):
                    leftLine = True
                else:
                    leftLine = False
                if(rightSide < lightOffset):
                    rightLine = True
                else:
                    rightLine = False
                    
                if(leftLine is True and rightLine is False):
                    turn_45(-1)
                if(rightLine is True and leftLine is False):
                    turn_45(1)
                if(leftLine is True and rightLine is True):
                    turn_180()
                
                #END -------------------- Checking Light Sensor


                #BEGIN -------------------- Checking Magnetometer
                mag = mpu9250.readMagnet()
                #print (" mx = " , ( mag['x'] )," my = " , ( mag['y'] )," mz = " , ( mag['z'] ))
                magBreak = True
                if(((mag['x'] + mag['y'] + mag['z']) == 0) and mode == 0 and magBreak == False):
                    BrickPi.MotorSpeed[PORT_B] = 0
                    BrickPi.MotorSpeed[PORT_C] = 0
                    magFound = True
                    tempMagnetReading = mag['x'] + mag['y'] + mag['z']
                    print("===========MAGNET - STOPPING!!!!!=========================")
                    print("===========MAGNET - STOPPING!!!!!=========================")
                    print("===========MAGNET - STOPPING!!!!!=========================")
                    count = 0
                    while(mode == 0 and count < 3):
                        if(conn2.poll(.05) == 1):
                            mode = conn2.recv()
                        else:
                            mode = conn2.recv()
                        
                        if(time.time() > startTime + 4):
                            BrickPi.MotorSpeed[PORT_B] =  60#Set the speed of MotorA (-255 to 255)
                            BrickPi.MotorSpeed[PORT_C] = -60#Set the speed of MotorB (-255 to 255)
                            ot = time.time()
                            wait = 1  
                            while(time.time() - ot < wait and lean is 0):    #running while loop for 3 seconds
                                BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
                                time.sleep(.02)
                            motorStop()
                            startTime = time.time()
                            scanFlag = False
                            count = count + 1
                            
                                     
                        time.sleep(.1)
                #END -------------------- Checking Magnetometer
                
                count = 0
                if(mode is 1 and time.time() > timer):
                    while(flag is False and count < 4):
                        turn_90(-1)
                        if(conn4.poll(.5) == 1):
                            lean = conn4.recv()
                            if(lean is not 0):
                                flag = True
                        if(conn2.poll(.01) == 1):
                            mode = conn2.recv()
                    
                        count = count + 1
                    timer = time.time() + 20
                    
                
                #BEGIN -------------------- Turning
                else:
                    result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                    if not result :
                        dist = BrickPi.Sensor[US1_port_number]
                    
                    if (dist < 15 and dist > 0 and preDist < 25 and preDist > 0):
                        BrickPi.MotorSpeed[PORT_B] = 0  #Set the speed of MotorA (-255 to 255)
                        BrickPi.MotorSpeed[PORT_C] = 0
                        BrickPiUpdateValues()
                        time.sleep(.25)

                        reverse()
                        time.sleep(.25)
                        turn_45(1)
                        time.sleep(.25)
                                        
                        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                        if not result :
                            compareRight = BrickPi.Sensor[US3_port_number]
                           
                        #print (distRight)
                        time.sleep(.25)
                        
                        turn_90(-1)
                        time.sleep(.25)
                        
                        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                        if not result :
                            compareLeft = BrickPi.Sensor[US2_port_number]
                        #print (distLeft)
                        time.sleep(.25)
                        
                        if compareLeft < compareRight: #turn left
                            turn_90(1)
                            turn_45(1)
                        else :
                            turn_45(-1)
                        time.sleep(.25)

                        speedL = 1
                        speedR = 1
                        
                    preDist = dist
                #END -------------------- Turning
            prevMode = mode
            prevTargetFound = targetFound
            time.sleep(.005)
       
#------------------NAVIGATION--------------------

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    parent_conn2, child_conn2 = Pipe()
    parent_conn3, child_conn3 = Pipe()
    parent_conn4, child_conn4 = Pipe()
    parent_conn5, child_conn5 = Pipe()
    
    startup()
    
    s = Process(target=opencv, args=(parent_conn,parent_conn2,parent_conn4,child_conn5,))
    r = Process(target=cannon, args=(child_conn,parent_conn3,))
    q = Process(target=navigation, args=(child_conn2,child_conn4, child_conn5,))
    c = Process(target=fireCannon, args=(child_conn3,))

    s.start()
    r.start()
    q.start()
    c.start()

    
    s.join()
    r.join()
    q.join()
    c.join()
    
