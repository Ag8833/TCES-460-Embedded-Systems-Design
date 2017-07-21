 #!/usr/bin/env python
# Karan Nayan
# Initial Date: July 11, 2013
# Updated:      July 11, 2013
# Updated:      Oct  28, 2016  Shoban
#
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)
#
# This is an example of controlling the rotation of motors using encoders
# Connect the LEGO Motors to Motor ports MA and MB.
#
# You can learn more about BrickPi here:  http://www.dexterindustries.com/BrickPi
# Have a question about this example?  Ask on the forums here:  http://forum.dexterindustries.com/c/brickpi

#For the code to work - sudo pip install -U future

from __future__ import print_function
from __future__ import division
from builtins import input

# the above lines are meant for Python3 compatibility.
# they force the use of Python3 functionality for print(), 
# the integer division and input()
# mind your parentheses!

from BrickPi import *   #import BrickPi.py file to use BrickPi operations

BrickPiSetup()  # setup the serial port for sudo su communication
BrickPiSetupSensors()       #Send the properties of sensors to BrickPi
#print('Before update')
BrickPiUpdateValues()

motor1 = PORT_A
BrickPi.MotorEnable[motor1] = 1 #Enable the Motor A
BrickPi.MotorEnable[PORT_D] = 1
"""
Pass the arguments in a list. 
If a single motor has to be controlled then the arguments should be passed like 
elements of an array,e.g, motorRotateDegree([255],[360],[PORT_A]) where power=255 and
angle=360 for the motor connected at Port A
"""
power=[255]
deg=[720] # pwr: 255, deg: 720, ecode: .06, encode_stop: 0.1
port=[PORT_A]

BrickPiUpdateValues()

degree = 720
init = BrickPi.Encoder[PORT_A]
final = BrickPi.Encoder[PORT_A]

BrickPi.MotorSpeed[PORT_A] = power[0]
BrickPi.MotorSpeed[PORT_D] = -power[0] 

BrickPiUpdateValues()

dif = 0
#print('initial ', init)
while dif < degree:
    BrickPiUpdateValues()
    
    final = BrickPi.Encoder[PORT_A] #wait
    dif = final - init
    print (dif)
    time.sleep(0.001)
    

#BrickPi.MotorSpeed[PORT_A] = -255  
#BrickPi.MotorSpeed[PORT_B] = 255
#BrickPiUpdateValues()

BrickPi.MotorSpeed[PORT_A] = 0  
BrickPi.MotorSpeed[PORT_D] = 0
BrickPiUpdateValues()

#print ( BrickPi.Encoder[PORT_A] %720 )
#print ('Before function')
#motorRotateDegree(power,deg,port, .12, .21) #This read the encoder values every 100 ms (default). Not that accurate but not very processor intensive
#print ( BrickPi.Encoder[PORT_A] %720 )
#motorRotateDegree(power,deg,port,.05)	#This read the encoder values every 50 ms. A little more accurate 
#motorRotateDegree(power,deg,port,0)	#This read the encoder values without any delay. Most accurate but take a lot of processing power
#motorRotateDegree(power,deg,port,0,.04)	#This read the encoder values without any delay. The time to rotate the motors in the opposite direction when stopping is specified in the last argument. Useful when free running the motors

"""
If multiple motors have to be controlled then the parameters for running each motor must be passed
as the elements of an array,e.g, motorRotateDegree([255,100],[360,30],[PORT_A,PORT_B]) where 
power=255 and angle=30 are for motor at PORT_A and power=100 and angle=30 are for motor at PORT_B.
It can be used similarly for any number of motors.
"""
#time.sleep(1)
#power=[255,30]
#deg=[360,-180]
#port=[PORT_A,PORT_B]
#motorRotateDegree(power,deg,port)
