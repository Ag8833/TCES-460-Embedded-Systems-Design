#Code for autonomous robot to navigate, recognize drones/cars, and track/shoot at them
#Created by: Reagan Stovall and Andrew Gates

import time
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


#------------------Cannon Setup--------------------
left = GPIO.input(LLSPin)  # LeftBounds
right = GPIO.input(RLSPin) # RightBounds
up = GPIO.input(ULSPin)    # LeftBounds
down = GPIO.input(DLSPin)  # RightBounds

#------------------Cannon Setup--------------------


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

def fireCannon():
    while(1):
        shotFired = GPIO.LOW
        laserOn()
        GPIO.output(FCPin, GPIO.HIGH) #fire on
        time.sleep(.75)
        while (shotFired == GPIO.LOW):
            shotFired = GPIO.input(SFPin)
                    
        GPIO.output(FCPin, GPIO.LOW) #fire off
        laserOff()
        time.sleep(.01)
    print("fire")    
        		
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
        up = GPIO.input(ULSPin) # LeftBounds
        down = GPIO.input(DLSPin) # RightBounds
        while (up == GPIO.LOW):
            
            lookUp()
            time.sleep(.01)
            stopY()
            up = GPIO.input(ULSPin) # LeftBounds
            
        starty_time = time.time()
        lookDown()
        time.sleep(.1)
        stopY()
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        stopX()
        GPIO.cleanup()        
   
try:
    while (1):
        startup()
        fireCannon()
        fireTurret()    
    
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
    stopX()
    GPIO.cleanup()  
