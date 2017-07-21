

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

leftLineSensor = 6 # BCM pin for the l line sensor
rightLineSensor = 5 # BCM pin for the r line sensor

leftLine = False
rightLine = False
updateTime = time.time()


def readLeftLineSensor():
    
    GPIO.setup(leftLineSensor, GPIO.OUT)
    GPIO.output(leftLineSensor, GPIO.HIGH)
    time.sleep(.00001)#sleep for 10 microseconds
    GPIO.setup(leftLineSensor, GPIO.IN)
    leftLS = GPIO.input(leftLineSensor)
    thisTime = time.time()
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
    while((rightLS == GPIO.HIGH)and((time.time()-thisTime)<(.003))):
        rightLS = GPIO.input(rightLineSensor)
    diff = time.time()-thisTime

    return diff
    
while(1):
    leftSide = readLeftLineSensor()
    rightSide = readRightLineSensor()
    if(leftSide < .0027):
        leftLine = True
    
    else:
        leftLine = False
    if(rightSide < .0027):
        rightLine = True
       
    else:
        rightLine = False
    if(leftLine is True and rightLine is False):
        print("right motor off")
        time.sleep(.5)
    if(rightLine is True and leftLine is False):
        print("left motor off")
        time.sleep(.5)
    if(leftLine is True and rightLine is True):
        print("stop and turn left until both left and right are false")
        time.sleep(1)    
    #print(leftLine, rightLine)
    print(leftSide, rightSide)

