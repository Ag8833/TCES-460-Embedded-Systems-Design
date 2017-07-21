#Code for autonomous robot to navigate, recognize drones/cars, and track/shoot at them
#Created by: Regan Stovall and Andrew Gates

from __future__ import print_function
from __future__ import division
from builtins import input
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
from multiprocessing import Process, Pipe
import RPi.GPIO as GPIO
import time
import multiprocessing
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

#--------------------MPU--------------------
import FaBo9Axis_MPU9250
import sys

mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#--------------------MPU--------------------

#------------------TURN FUNCTIONS--------------------
def turn_180(right = 1):
    BrickPi.MotorSpeed[PORT_A] = 100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = -100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    while(time.time() - ot < .87):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motorStop()

def turn_90(right = 1):
    BrickPi.MotorSpeed[PORT_A] = 100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = -100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .72/2 + 0.11  # Half of 180
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motorStop()

def turn_45(right = 1):
    BrickPi.MotorSpeed[PORT_A] = 100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = -100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .72/4 + 0.11  # Half of 90
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motorStop()

def reverse():
    BrickPi.MotorSpeed[PORT_A] = 50
    BrickPi.MotorSpeed[PORT_D] = 50
    ot = time.time()
    wait = .72/4 + 0.11  # Half of 90
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)
    motorStop()
    
def motorStop(): # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()
    
#------------------TURN FUNCTIONS--------------------

#------------------BRICKPI--------------------
BrickPiSetup()  # setup the serial port for communication

LS_port_number = PORT_1
port_number = PORT_4	        # Define the port number here.
US2_port_number = PORT_3	# Define the port number here.  
US3_port_number = PORT_2	# Define the port number here.  

BrickPi.SensorType[port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
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

#Globals
preDist = 0
dist = 0
distL = 0
distR = 0
speedL = 1
speedR = 1
#------------------BRICKPI--------------------

#------------------CANNON--------------------
cannonShots = 4
turretShots = 6

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
FTPin = 11 # BCM pin fire turret
LZRPin = 20 # BCM Laser

ButtonPin = 4 # BCM pin for reload

#Globals
left = GPIO.LOW
right = GPIO.LOW
up = GPIO.LOW
down = GPIO.LOW

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
GPIO.setup(FTPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(LZRPin, GPIO.OUT) # Sets Lazer on or off 

#Initial states for Motors:
GPIO.output(X1Pin, GPIO.LOW)
GPIO.output(X2Pin, GPIO.LOW)
GPIO.output(Y1Pin, GPIO.LOW)
GPIO.output(Y2Pin, GPIO.LOW)
GPIO.output(FCPin, GPIO.LOW)
GPIO.output(FTPin, GPIO.LOW)
GPIO.output(LZRPin, GPIO.LOW)

# Functions
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
        		
def fireTurret():
    laserOn()
    GPIO.output(FTPin, GPIO.HIGH)
    time.sleep(.075)	
    GPIO.output(FTPin, GPIO.LOW)
    laserOff()
		
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
def opencv(conn, conn2):
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
    cannonShots = 4
    turretShots = 6
    shotFired = 0

    #Setup cascades for drones, cars and reloads
    drone_cascade = cv2.CascadeClassifier('sphere_cascade.xml')
    car_cone_cascade = cv2.CascadeClassifier('20_stage_normal_cone.xml')
    reload_cone_cascade = cv2.CascadeClassifier('17_stage_upside_down_cone.xml')

    try:
        while(1):
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                print('IN OPENCV')
                #Capture the image from the frame, and convert to gray
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                #Scan the image for drones, cars and reloads
                drones = drone_cascade.detectMultiScale(gray, 1.2, 2)
                cars = car_cone_cascade.detectMultiScale(gray, 1.2, 2)
                reloads = reload_cone_cascade.detectMultiScale(gray, 1.05, 1)

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
                            conn.send(cars)
                            carFound = True
                            droneFound = False

                    if(droneFound == False and carFound == False):
                        conn.send(0)
                #END -------------------- Defense Mode

                #BEGIN -------------------- Reload Mode
                else:
                    for (x,y,w,h) in reloads:
                        cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
                        roi_gray = gray[y:y+h, x:x+w]
                        roi_color = image[y:y+h, x:x+w]
                        
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
                        cannonShots = 4
                        turretShots = 6
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
    #Centerpoints of the screen
    center_x = 75
    center_y = 75
    #Cannon/Turret ammunition
    cannonShots = 4
    turretShots = 6
    startTime = time.time()

    try:
        while(1):
            conn.send(0)
            target = conn.recv()
            #target = 0: Inactive 
            if(target == 0):
                print("Cannon Inactive")
            #target = 3: Restarting Defense Mode
            if(target == 3):
                cannonShots = 4
                turretShots = 6
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
                        if(abs(x_offset) < 10 and abs(y_offset < 10) and cannonShots > 0 and time.time() > startTime):
                            print ("CANNON fire!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            conn.send(1)
                            conn3.send(1)
                            startTime = time.time() + 2
                            cannonShots -= 1
                            
                        #Otherwise track the target    
                        trackX(x_offset)
                        trackY(y_offset)
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
                        if(abs(x_offset) < 10 and abs(y_offset < 10) and turretShots > 0):
                            print ("TURRET fire!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            conn.send(2)
                            fireTurret()
                            turretShots -= 1
                            
                        #Otherwise track the target    
                        trackX(x_offset)
                        trackY(y_offset)
             #END -------------------- Drone Tracking
        
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        conn.close()
        cannonStop()
        GPIO.cleanup() # cleanup all GPIO
#------------------CANNON--------------------

        
#------------------STARTUP--------------------
def startup():
    find_start_X()
    find_start_Y()

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
            
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        stopX()
        GPIO.cleanup()

#Function to set the Y position of Cannon to neutral
def find_start_Y():
    try:
        down = GPIO.input(DLSPin) # LeftBounds
        while (down == GPIO.LOW):
            lookDown()
            time.sleep(.01)
            stopY()
            down = GPIO.input(DLSPin) # LeftBounds
            
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        stopY()
        GPIO.cleanup()       
#------------------STARTUP--------------------

#------------------NAVIGATION--------------------
def navigation(conn2):
    preDist = 0
    dist = 0
    distL = 0
    distR = 0
    preDistL = 0
    preDistR = 0
    speedL = 1
    speedR = 1
    while(1):
        #if(conn2.poll(.5) == 1):
        #    mode = conn2.recv()
        #else:
        mode = conn2.recv()
        #mode = 1
        if(mode == 0):
            print("In navigation - DEFENSE MODE")
        else:
            print("In navigation - RELOAD MODE")
            BrickPi.MotorSpeed[PORT_A] = 0  #Set the speed of MotorA (-255 to 255)
            BrickPi.MotorSpeed[PORT_D] = 0
        BrickPiUpdateValues()
        distL = BrickPi.Sensor[US2_port_number]
        distR = BrickPi.Sensor[US3_port_number]
        print("Front - ", dist, ", Left - ", distL, ", Right - ", distR)
                
        #BEGIN -------------------- Driving Correction
        if(abs(distL - distR) < 10):
            BrickPi.MotorSpeed[PORT_A] = 52  #Set the speed of MotorA (-255 to 255)
            BrickPi.MotorSpeed[PORT_D] = 50
            BrickPiUpdateValues()
        elif(distL < distR):
            if(speedR <= 1):
                speedR = 1
            else:
                speedR = speedR - 1

            speedL = speedL + 1
                
            if(speedL > 100):
                speedL = 100
            
            BrickPi.MotorSpeed[PORT_A] = 52
            BrickPi.MotorSpeed[PORT_D] = 50 + speedL
        else:
            if(speedL <= 1):
                speedL = 1
            else:
                speedL = speedL - 1

            speedR = speedR + 1
            
            if(speedR > 100):
                speedR = 100
            
            BrickPi.MotorSpeed[PORT_A] = 52 + speedR
            BrickPi.MotorSpeed[PORT_D] = 50


        print("speedL = ", speedL, ", speedR = ", speedR)
        preDistL = distL
        preDistR = distR
        #END -------------------- Driving Correction

        #BEGIN -------------------- Checking Magnetometer
        mag = mpu9250.readMagnet()
        #print (" mx = " , ( mag['x'] )," my = " , ( mag['y'] )," mz = " , ( mag['z'] ))
        
        if(((mag['x'] + mag['y'] + mag['z']) == 0) and mode == 0):
            BrickPi.MotorSpeed[PORT_A] = 0
            BrickPi.MotorSpeed[PORT_D] = 0
            magFound = True
            tempMagnetReading = mag['x'] + mag['y'] + mag['z']
            print("MAGNET - STOPPING")
            while(tempMagnetReading == 0 and mode == 0):
                mag = mpu9250.readMagnet()
                #print (" mx = " , ( mag['x'] )," my = " , ( mag['y'] )," mz = " , ( mag['z'] ))
                #print("waiting1")
                tempMagnetReading = mag['x'] + mag['y'] + mag['z']

                if(conn2.poll(.5) == 1):
                    mode = conn2.recv()
                else:
                    mode = conn2.recv()
                #if(mode == 2):
                #    startup()
                time.sleep(.1)
        #END -------------------- Checking Magnetometer

        #BEGIN  -------------------- Checking Light Sensor
        if(BrickPi.Sensor[LS_port_number] < 550):
        #turn_45(-1)
            while(BrickPi.Sensor[LS_port_number] < 550):
                BrickPi.MotorSpeed[PORT_A] = 0
                BrickPi.MotorSpeed[PORT_D] = 0

                #if(conn2.poll(.5) == 1):
                #    mode = conn2.recv()
            print("WHITE LINE - STOPPING")
        #END -------------------- Checking Light Sensor

        #BEGIN -------------------- Turning
        else:
            result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
            if not result :
                dist = BrickPi.Sensor[port_number]

            if (dist < 25 and dist > 0 and preDist < 25 and preDist > 0):
                BrickPi.MotorSpeed[PORT_A] = 0  #Set the speed of MotorA (-255 to 255)
                BrickPi.MotorSpeed[PORT_D] = 0
                BrickPiUpdateValues()
                time.sleep(1)

                reverse()
                time.sleep(.5)

                turn_90(1)
                time.sleep(.5)
                
                result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                if not result :
                    distRight = BrickPi.Sensor[port_number]
                print (distRight)
                time.sleep(1)
                
                turn_180(-1)
                time.sleep(.5)
                
                result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                if not result :
                    distLeft = BrickPi.Sensor[port_number]
                print (distLeft)
                time.sleep(1)
                
                if distLeft <= distRight:
                    turn_180()
                time.sleep(1)

                speedL = 1
                speedR = 1
                
            preDist = dist
        #END -------------------- Turning
    time.sleep(.1)
#------------------NAVIGATION--------------------

if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    parent_conn2, child_conn2 = Pipe()
    parent_conn3, child_conn3 = Pipe()

    #startup()
    
    s = Process(target=opencv, args=(parent_conn,parent_conn2,))
    r = Process(target=cannon, args=(child_conn,parent_conn3,))
    q = Process(target=navigation, args=(child_conn2,))
    c = Process(target=fireCannon, args=(child_conn3,))

    s.start()
    r.start()
    q.start()
    c.start()

    
    s.join()
    r.join()
    q.join()
    c.join()
    
