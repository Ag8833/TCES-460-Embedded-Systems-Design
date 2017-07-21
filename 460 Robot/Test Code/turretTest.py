import time
import RPi.GPIO as GPIO

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
Y2Pin = 21 # BCM pin y-axis Motor 4AY Grey
FCPin = 16  # BCM pin fire cannon
FTPin1 = 13 # BCM pin fire turret
FTPin2 = 26 # BCM pin fire turret

LZRPin = 20 # BCM Laser

ButtonPin = 4 # BCM pin for reload

# Globals
left = GPIO.LOW
right = GPIO.LOW
up = GPIO.LOW
down = GPIO.LOW

# set up the GPIO channels - 5 input and 5 output
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

# Initial states for Motors:
GPIO.output(X1Pin, GPIO.LOW)
GPIO.output(X2Pin, GPIO.LOW)
GPIO.output(Y1Pin, GPIO.LOW)
GPIO.output(Y2Pin, GPIO.LOW)
GPIO.output(FCPin, GPIO.LOW)
GPIO.output(FTPin1, GPIO.LOW)
GPIO.output(FTPin2, GPIO.LOW)

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

def fireCannon():
    shotFired = GPIO.LOW
    laserOn()
    while ( shotFired == GPIO.LOW):
        GPIO.output(FCPin, GPIO.HIGH) #fire on
        shotFired = GPIO.input(SFPin)
    time.sleep(.5) #ratchet up the next shot    
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
    GPIO.output(FTPin, GPIO.LOW)
    laserOff()
    time.sleep(0.1)	

def laserOn():
    GPIO.output(LZRPin, GPIO.HIGH)

def laserOff():
    GPIO.output(LZRPin, GPIO.LOW)

def trackX(xOffset):
    print ("tracking X")
    #trackSpeedX = abs(xOffset)/10000
    checkX()
    if (xOffset > 0):
        if(right == GPIO.HIGH):
            print ("right limit")
        else:   
            lookRight()
            time.sleep(.015)
    else:
        if(left == GPIO.HIGH):
            print ("left limit")
        else:   
            lookLeft()
            time.sleep(.015)
    stopX()
    
def trackY(yOffset):
    print ("tracking Y")
    checkY()
    if (yOffset > 0):
        if(up == GPIO.HIGH):
            print ("upper limit")
        else:
            lookUp()
            time.sleep(.012)
    else:
        if(down == GPIO.HIGH):
            print ("lower limit")
        else:
            lookDown()
            time.sleep(.015)
    stopY()

    
if __name__ == '__main__':
    while(1):
        fireTurret1()
        #fireCannon()
        time.sleep(1)
        fireTurret2()
        time.sleep(1)
        print('looping')
   
 
