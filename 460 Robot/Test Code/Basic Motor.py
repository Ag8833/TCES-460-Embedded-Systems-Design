#!/usr/bin/env python

#####################################################################################
#
# Brickpibot.py is a port of GoPiGo robot_controller with just the functions needed
# to run the browser stream robot. Since BrickPi makes it easier to address motors
# by a port, instead of manually controlling PWM it is much simpler.
#
# History
# ------------------------------------------------
# Author        Date            Comments
# Peter Lin     27 Dec 14       Script uses global for setting left/right power
#
# Notes: On a slow Wifi network, there's significant lag, so it may not be as
#        responsive as using a real joystick that connects directly. It might
#        work better if the robot is setup as an adhoc (aka hotspot) network
#        with the browser connecting directly to it.
#
#        The script is self contained and could be used in other projects, but
#        it might need additional functionality.
#
#        If you run into problems, a quick and easy way to debug it is to
#        uncomment the print statements to see what power values are passed to
#        the functions. When the script starts up, it will setup the BrickPi by
#        calling BrickPiSetup() and BrickPiSetupSensors(). Change the ports to
#        the ports you're using or plug the motors into port B and D.
#
#####################################################################################

from BrickPi import *
import time

#Move the simplebot depending on the command
def move_bot(m1s,m2s):
	BrickPi.MotorSpeed[motor1] = m1s 
	BrickPi.MotorSpeed[motor2] = m2s




motor1=PORT_A	# motor1 is on PORT_B
motor2=PORT_B	# motor2 is on PORT_C
BrickPi.MotorEnable[motor1] = 1 #Enable the Motor1
BrickPi.MotorEnable[motor2] = 1 #Enable the Motor2 

#BrickPiSetup()


#move_bot(250,250)
#BrickPiUpdateValues()

#BrickPi.Timeout=10000	#Set timeout value for the time till which to run the motors after the last command is pressed
#BrickPiSetTimeout()

BrickPiSetup()  # setup the serial port for sudo su communication
BrickPiSetupSensors()       #Send the properties of sensors to BrickPi

move_bot(255, 255)
BrickPiUpdateValues()

BrickPi.Timeout=10000	#Set timeout value for the time till which to run the motors after the last command is pressed
BrickPiSetTimeout()

move_bot(0, 0)
BrickPiUpdateValues()

"""
Pass the arguments in a list. 
If a single motor has to be controlled then the arguments should be passed like 
elements of an array,e.g, motorRotateDegree([255],[360],[PORT_A]) where power=255 and
angle=360 for the motor connected at Port A
"""
power=[150]
deg=[90]
port=[PORT_A]

motorRotateDegree(power,deg,port, 0)		#This read the encoder values every 100 ms (default). Not that accurate but not very processor intensive
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
