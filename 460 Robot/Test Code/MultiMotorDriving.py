

#!/usr/bin/env python

# Initial Date: July 11, 2013
# Last Updated: July 11, 2013
# http://www.dexterindustries.com/

# Written by Karen Bang, Jane Maeng, and Yeda Min - students at Miss Porter's School 
# Rolly Noel performed major rennovations January, 2017
#   including replacing backing up to correct overshoot, with slowing down slightly before the target is reached;
#   also learning from previous over/undershoots at each power setting so can nail it next time;
#   as well as renaming it from LEGO_Multi_Motor_Encoder_Test_ABK.
#
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)
#
# This intends to accurately & reliably control the rotation of LEGO motors attached to a Raspberry Pi
#   via a Dexter Industries BrickPi

# BrickPi's first Arduino runs motor ports A & b, as well as sensor ports 1 & 2. Arduino 2 does the rest of them.
# You may get better performance (certainly fewer communication errors with the BrickPi, and less work using my version
#   of the BrickPi.py driver) if you place all your motors and sensors on the same Arduino (if possible).
# Rolly's version of BrickPi.py won't bother to poll (triggered by BrickPi.UpdateValues()) an Arduino without gear.

from __future__ import print_function
from __future__ import division

import time
from BrickPi import *  #import BrickPi.py file to use BrickPi operations

global inArray

debug = False
    
# CALIBRATE THE FOLLOWING ENTRIES FOR YOUR ROBOT 
#  - These are for my 1 kg robot with 2 driving wheels & a dolly wheel
#      (at a power of 30, turning off a motor 2 tics prior to the destination stopped on the mark)
recordOfStops = {30:[2,0], 100:[8,0], 255:[41,0]}  #key is power, values for each motor 
                                                   # are pairs of leads & average tics over/under target

#Determine how many tics ahead of the destination tic count to turn off the motor (so it will coast to the right point)
def chooseALead (power, degrees):
    powerToDo = abs(power)
    ticsToTravel  = abs(degrees) * 2
    if powerToDo in recordOfStops:
        n = 0
        totals = 0
        readings = recordOfStops[powerToDo]
        for values in readings:     #values are pairs of tic count leads and early/late results
            totals += values
            n +=1
        ticsToLeadWith = int(totals // (n // 2)) 
    else:
        n = 0
        powers = 0
        results = 0
        for power in recordOfStops:
            powers += power
            for lead, overUnder in zip(recordOfStops[power][::2], recordOfStops[power][1::2]):
                results += lead + overUnder
                n += 1
        averagePower = powers // n
        averageResults = results // n
        ticsToLeadWith =  int(averageResults * powerToDo // averagePower)
        #print ("Average power:%d, average results:%d, this power:%d, ad hoc ticsToLeadWith: %d" \
        #  % (averagePower, averageResults, powerToDo, ticsToLeadWith))
    ticsToLeadWith = abs(ticsToLeadWith)
    if ticsToLeadWith > ticsToTravel:  #In case I calc'd to start stopping before I started,
        ticsToLeadWith = int(ticsToTravel - 0.45 * powerToDo)    # better make it more reasonable
        #DON'T ADJUST ABOVE FACTOR (IE .45) WITH DEBUG ON AS IT SCREWS THE TIMING)
    #print ("ticsToLeadWith: %d" % ticsToLeadWith)
    return ticsToLeadWith
        

#Keep track, for power level used, of this lead time, and whether I stopped before or after the target tic cout                   
def addALead (power, lead, overUnder):
    if power not in recordOfStops:
        if len(recordOfStops) > 100: return	# 100 entries should do
        recordOfStops[power] = [lead, -overUnder]
    else: recordOfStops[power] += [lead, -overUnder]
    #print (recordOfStops[power])


"""
Pass the arguments in a list. 
If a single motor has to be controlled then the arguments should be passed like 
elements of an array,e.g, motorRotateDegree([255], [360], [PORT_A]) where power=255 and
angle=360 for the motor connected at Port A
For multi motor calls: motorRotateDegree([255, 255], [1000, 1000], [PORT_A, PORT_B]) 
"""
#power=[255]
#deg=[360]
#port=[PORT_A]
#motorRotateDegree(power,deg,port)      #This reads the encoder values every 100 ms (default). Not that accurate but not very processor intensive
#motorRotateDegree(power,deg,port,.05)	#This reads the encoder values every 50 ms. A little more accurate 
#motorRotateDegree(power,deg,port,0)	#This reads the encoder values without any delay. Most accurate but take a lot of processing power

# An alternative to DI's motorRotateDegree
def motorRotateDeg (power, deg, port, sampling_time=.01):
    """Rotate the selected motors by specified degree
    Args:
      power    : an array of the power values at which to rotate the motors (0-255)
      deg    : an array of the angle's (in degrees + or -) by which to rotate each of the motor
      port    : an array of the port's on which the motor is connected
      sampling_time  : (optional) the rate(in seconds) at which to read the data in the encoders
        Returns:
        max encoder tics between motors (if the power settings are the same, & whilst both motors are running)
        Usage:
        Pass the arguments in a list. if a single motor has to be controlled then the arguments should be
      passed like elements of an array,e.g, motorRotateDegree([255],[360],[PORT_A]) or
      motorRotateDegree([255,255],[360,360],[PORT_A,PORT_B])
    """
      
    numberOfMotors = len(power)             #Number of motors being used
    motorRunning = [0] * numberOfMotors     #For each Lego motor, whither its running (initially not)
    initTics = [0] * numberOfMotors         #Tic count (is degrees * 2) when function called
    currentTics = [0] * numberOfMotors      #Tic count thruout the operation
    startToStop = [0] * numberOfMotors      #Tic count at which to shut motor down so will coast to stop on the mark
    targetTics = [0] * numberOfMotors       #Tic count of the mark
    previousTics = [0] * numberOfMotors
    stallSpottedTime = [0] * numberOfMotors

    currentPower = [0] * numberOfMotors	    #From BrickPi.MotorSpeed
    
    doingPIC = False
    delta = 0
    gain = 0.018
    idelta = 0.0
    alpha = 10
    powerMod = 0
    maxWheelSpeedDiff = 0
    lowestPowerSetting = 20 #11 is the stall speed

    #Init motors
    nowTime = time.time()
    if (numberOfMotors == 2) and (abs(power[0]) == abs(power[1])):
        doingPIC = True
        lowestPowerSetting = int(abs(power[0]) * 0.75)   #Don't want motors to wobble
        maxWheelSpeedDiff = 0
        lastTimeChangedSpeed = nowTime
    while BrickPiUpdateValues(): pass           	#Grab current values (ignore invalid updates)
    for i in range(numberOfMotors):
        initTics[i] = BrickPi.Encoder[port[i]] 		#Initial reading of the encoder
        targetTics[i] = initTics[i] + (deg[i] * 2) 	#When the motor has to be stopped;One encoder tic counts for 0.5 degrees
        leadTics = chooseALead (power[i], deg[i])   #Determine how many tics to lead the stopping point  
        if targetTics[i] > initTics[i]:   			#find out when to start-to-stop (so will coast to the mark)
            startToStop[i] = targetTics[i] - leadTics  # (accounting for whether we're coming or going)
        else: startToStop[i] = targetTics[i] + leadTics
        if debug: print ("Motor %d, encoder init: %d, start-to-stop: %d, encoder target: %d, power: %d" %   \
                    (port[i], initTics[i], startToStop[i] ,targetTics[i], power[i]))
        power[i] = abs(power[i])
        #For running clockwise and anticlockwise
        if deg[i] > 0:
           BrickPi.MotorSpeed[port[i]] = power[i]
           motorRunning[i] = True
           BrickPi.MotorEnable[port[i]] = 1         #Ensure Motor enabled
        elif deg[i]<0:
           power[i] *= -1					        #So power set for clockwise or anticlockwise
           BrickPi.MotorSpeed[port[i]] = power[i]
           motorRunning[i] = True
           BrickPi.MotorEnable[port[i]] = 1         #Ensure Motor enabled
        else: BrickPi.MotorSpeed[port[i]] = 0
    result = BrickPiUpdateValues()                  #Ask BrickPi to start motors (so they start at same time)

    #Loop to run motors
    while True:
        if sampling_time: time.sleep(sampling_time)  #if called for, sleep for the sampling time given (default:10 ms)
        lastTime = nowTime
        nowTime = time.time()
        result = BrickPiUpdateValues()              #Ask BrickPi to update values for sensors/motors
        if debug:
            if result: print ("Result of Update Values: %d" % result)
        if not result:
            #Got a good update
            for i in range(numberOfMotors):         #Do for each of the motors
                if not motorRunning[i]: continue    #If motor not running, skip on               
                currentPower[i] = BrickPi.MotorSpeed[port[i]]
                previousTics[i] = currentTics[i]
                currentTics[i] = BrickPi.Encoder[port[i]] #Grab tick count
                while (currentTics[i] == 0):         #Ignore encoder readings of 0    
                    if debug and previousTics[i]:
                        print ("Motor " + str(port[i]) + " got a 0 encoder reading, " + ":".join(hex(x) for x in Array[:20]))
                        return
                    if sampling_time: time.sleep(sampling_time) #Wait for a bit if called for
                    result = BrickPiUpdateValues()
                    previousTics[i] = currentTics[i]
                    currentTics[i] = BrickPi.Encoder[port[i]]
                #Check for stalled motor
                if previousTics[i] == currentTics[i]:
                    if stallSpottedTime[i] == 0: stallSpottedTime[i] = nowTime
                    elif (nowTime - stallSpottedTime[i]) > 0.5: #if same for 1/2 sec:
                        if debug: print("Motor %d stalled @ %d tics for 0.5 secs, @ power: %d" \
                          % (i, currentTics[i], currentPower[i]))
                        stallSpottedTime[i] = 0     #One report of this per 1/2 sec will do
                        #DON'T KNOW HOW TO RECTIFY THIS!!!
                else: stallSpottedTime[i] = 0
                #Check if motor running & time to start stopping it
                if (deg[i] > 0 and (startToStop[i] <= currentTics[i])) or (deg[i] < 0 and (startToStop[i] >= currentTics[i])):
                    #This motor has reached its goal
                    motorRunning[i] = False         #indicate its not running
                    BrickPi.MotorSpeed[port[i]] = 0
                    #print("motor done:", i)
                    BrickPiUpdateValues()
                    #If was doing PIC, and other motor still running, better add in its remaining tic count to diff
                    if doingPIC:
                        if BrickPi.MotorSpeed[port[i - 1]]:
                            penaltyForTardiness = abs(startToStop[i - 1]) - abs(BrickPi.Encoder[port[i - 1]])
                            if penaltyForTardiness > 0:
                                print ("penalty for tardiness added to max speed differential: ", \
                                  penaltyForTardiness)
                                print ("stopped motor: %d, tardy motor:%d" \
                                  % (BrickPi.Encoder[port[i]], BrickPi.Encoder[port[i - 1]]))
                                maxWheelSpeedDiff += penaltyForTardiness
                                # Ensure the following PIC code hadn't slowed down other motor
                                if abs(BrickPi.MotorSpeed[port[i - 1]]) < abs(power[i - 1]): 
                                    BrickPi.MotorSpeed[port[1 - i]] = power[i - 1]  #Bring it back up to speed 
        #If all the motors have already completed their rotation, then clean up & return
        if (all(e==False for e in motorRunning)):
            #Loop till all motors have come to a stop
            for i in range(numberOfMotors):         #Do for each of the motors
                if (previousTics[i] != BrickPi.Encoder[port[i]]):
                    previousTics[i] = BrickPi.Encoder[port[i]]
                    time.sleep(sampling_time)
                    BrickPiUpdateValues()
            for i in range(numberOfMotors):
                if debug: print ("Motor %d encoder done: %d" % (port[i], currentTics[i]))
                #Record our performance (for each motor) for this movement to help do better next time
                addALead (abs(power[i]), abs(targetTics[i] - startToStop[i]), currentTics[i] - targetTics[i])
            return maxWheelSpeedDiff                 #All done & OK
        
        #Use Proportional Integral Control on the Motors to keep them in Sync
        #  (I DON'T KNOW IF THIS EVER WAS PIC, BUT IT WORKS)
        if doingPIC and (all(e==True for e in motorRunning)):    #Ensure to do PIC & both motors still running
            if currentTics[0] == 0 or currentTics[1] == 0: continue  #Ensure don't base on bogus encoder readings
            delta = abs(currentTics[1] - initTics[1]) - abs(currentTics[0] - initTics[0]) #determine speed difference
            speedChangeTimeDelta = nowTime - lastTimeChangedSpeed
            if abs(delta) > maxWheelSpeedDiff:
                maxWheelSpeedDiff = abs(delta)	     #Keep track of max extent
                #print ("   %d msecs since last power change, new max delta: %d, motor 0 power: %d, motor 1 power: %d" \
                #  % (speedChangeTimeDelta * 1000, maxWheelSpeedDiff, BrickPi.MotorSpeed[port[0]], \
                #  BrickPi.MotorSpeed[port[1]]))
            #ignore if difference trivial
            if abs(delta) < (.01 * abs(power[1])): continue
            #Ignore if too soon to tell (motor inertia), unless difference is large
            #if (speedChangeTimeDelta < 0.005) and (abs(delta) < .50 * abs(power[1])): continue
            if speedChangeTimeDelta < 0.010: continue
            lastTimeChangedSpeed = nowTime      
            # THE FOLLOWING LINE IS THE GUTS OF PIC. powerMod is added to/subtracted from a motor's current speed/power
            #powerMod = int((power[0] * abs(delta) * gain) + idelta * gain) #Power Multiplier: the amount to change power setting
            powerMod = abs(delta) * 20
            if debug: print ("   %d Msec since last changed power, delta: %d, desired powerMod: %d" \
               % (speedChangeTimeDelta * 1000, delta, powerMod))
			
            if delta > 0:                            #Determine which is faster              
                #Motor 1 is going faster
                if debug: print ("M1 is faster by %d, m0 speed: %d,  m1 speed: %d, , power modifier: %d"  \
                  % (abs(delta), currentPower[0], currentPower[1], powerMod))  
                #Don't speed up motor 0 if its already at its original speed (SHOULDN'T EXCEEDE THAT???)
                if abs(currentPower[0]) >= abs(power[0]):
                    #Motor 0 CANNOT be sped up, so slow down motor 1
                    if abs(powerMod) < (abs(currentPower[1]) - lowestPowerSetting): #Ensure don't reverse or stop motor 1
                        #Slow down motor 1
                        if BrickPi.MotorSpeed[port[1]] < 0: BrickPi.MotorSpeed[port[1]] += powerMod
                        else: BrickPi.MotorSpeed[port[1]] -= powerMod
                    else:
                        if BrickPi.MotorSpeed[port[1]] < 0: BrickPi.MotorSpeed[port[1]] = -lowestPowerSetting # Do the best I can
                        else: BrickPi.MotorSpeed[port[1]] = lowestPowerSetting  # (too low a power & she'll stall)
                        if debug: print("   Not able to reduce motor 1 power by  %d, instead set to %d" \
                           % (powerMod, BrickPi.MotorSpeed[port[1]]))
                        continue                    #leave things alone for now   
                    if debug: print ("   Power modifier: %d, New motor 1 power: %d" % (powerMod, BrickPi.MotorSpeed[port[1]]))
                else:
                    #Motor 0 CAN be sped up
                    if abs(currentPower[0] + powerMod) <= abs(power[0]): #Don't allow it to exceede its original speed  
                        BrickPi.MotorSpeed[port[0]] +=  powerMod #Speed up motor 0      
                    else: BrickPi.MotorSpeed[port[0]] = power[0] # (to its max)
                    if debug: print ("   Power modifier: %d, New motor 0 power: %d" % (powerMod, BrickPi.MotorSpeed[port[0]]))   
            
            elif delta < 0:
                #Motor 0 is going faster
                if debug: print ("M0 is faster by %d, M0 power: %d, M1 power: %d, Power modifier: %d"  \
                  % (abs(delta), currentPower[0], currentPower[1], powerMod))
                #Don't speed up motor 1 if its already at its original speed (SHOULDN'T EXCEEDE THAT???)
                if abs(currentPower[1]) >= abs(power[1]):
                    #Motor 1 CANNOT be sped up, so slow down motor 0
                    if abs(powerMod) < (abs(currentPower[0]) - lowestPowerSetting): #ensure don't reverse or stop motor 0
                        #Slow down motor 0
                        if BrickPi.MotorSpeed[port[0]] < 0: BrickPi.MotorSpeed[port[0]] += powerMod
                        else: BrickPi.MotorSpeed[port[0]] -= powerMod
                    else:
                        if BrickPi.MotorSpeed[port[0]] < 0: BrickPi.MotorSpeed[port[0]] = -lowestPowerSetting # Do the best I can
                        else: BrickPi.MotorSpeed[port[0]] = lowestPowerSetting    # (too low a power & she'll stall)
                        if debug: print("   Not able to reduce motor 0 power by %d, instead set to %d" \
                          % (powerMod, BrickPi.MotorSpeed[port[0]]))
                        continue                    #leave things alone for now
                    if debug: print ("   Power modifier: %d, New motor 0 power: %d" % (powerMod, BrickPi.MotorSpeed[port[0]]))
                else:
                    #Motor 1 CAN be sped up
                    if abs(currentPower[1] + powerMod) <= abs(power[1]): #Don't allow it to exceede its original speed
                        BrickPi.MotorSpeed[port[1]] +=  powerMod #Speed up motor 1
                    else: BrickPi.MotorSpeed[port[1]] = power[1] # (to its max)
                    if debug: print ("   Power modifier: %d, New motor 1 power: %d" % (powerMod, BrickPi.MotorSpeed[port[1]]))   


