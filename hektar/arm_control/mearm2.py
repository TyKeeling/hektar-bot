import rospy
from hektar.msg import armCtrl, armPos, armMaster
from rosserial_arduino.msg import Adc
import kinematics
import rospy
import time
from math import pi

#init publisher to arduino

# Initilaize parameters
#   Base measurements not yet taken
    sweepMinBase = 145 
    sweepMaxBase = 49
    angleMinBase = -pi/4
    angleMaxBase = pi/4

    sweepMinShoulder = 372
    sweepMaxShoulder = 1117
    angleMinShoulder = pi/2
    angleMaxShoulder = 4 * pi/3

    sweepMinElbow = 496
    sweepMaxElbow = 807
    angleMinElbow = pi/6
    angleMaxElbow = pi

    angleMinGripper = pi/2
    angleMaxGripper = 0

    # Permitted error
    error = 0.25

    # Speed at which to move arm joints
    speed =  {
        "high" = 127
        "med" = 80
        "debug" = 40
    }

# takes in Master and Pos messages, outputs Ctrl message to be published
# params
# output
def move(master, position):
    nowBase = (position.basePos - sweepMinBase)/(sweepMaxBase - sweepMinBase) * (abs(angleMaxBase) + abs(angleMinBase))
    nowShoulder = (position.shoulderPos - sweepMinShoulder)/(sweepMaxShoulder - sweepMinShoulder) * (abs(angleMaxShoulder) + abs(angleMaxShoulder))
    nowElbow = (position.elbowPos - sweepMinElbow)/(sweepMaxElbow - sweepMinElbow) * (abs(angleMaxElbow) + abs(angleMinBase))
    angles = [0,0,0]

    opSpeed = speed["med"]
    
    if kinematics.solve(master.x, master.y, master.z, angles):
        newBase = angles[0]
    	newShoulder = angles[1]
    	newElbow = angles[2]
        msg = hektar.msg.armCtrl
        #set velocities nonzero if outside permitted error
        if abs(newBase - nowBase) > error:
            msg.baseVel = -opSpeed if newBase < nowBase else opSpeed
        else msg.baseVel = 0

        if abs(newElbow - nowElbow) > error:
            msg.elbowVel = -opSpeed if newBase < nowBase else opSpeed
        else msg.elbowVel = 0

        if abs(newShoulder - nowShoulder) > error:
            msg.shoulderVel = -opSpeed if newShoulder < nowShoulder else opSpeed
        else msg.shoulderVel = 0
    
    return msg

    def isReachable(master):
        """Returns True if the point is (theoretically) reachable by the gripper"""
    	radBase = 0
    	radShoulder = 0
    	radElbow = 0
    	return kinematics.solve(master.x, master.y, master.z, radBase, radShoulder, radElbow)

        
        



    



