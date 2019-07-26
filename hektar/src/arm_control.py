#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos, armTarget, armPotTargets
import time
from math import pi
import kinematics
import sys

pub = rospy.Publisher('arm_commands', armCtrl, queue_size=10)

# Initilaize parameters
sweepMinBase = 0
sweepMaxBase = 1024
offsetBase = -465

sweepMinShoulder = 315  # mechanical min
sweepMaxShoulder = 709  # mechanical max
offsetShoulder = -254 # 256 minus value at pi/2

sweepMinElbow = 210 # mechanical min
sweepMaxElbow = 540  # mechanical max
offsetElbow = -330 # offset for reading at angle 0

angleMinGripper = pi/2
angleMaxGripper = 0

class Arm:
  def __init__(self):

    self.integralBase = 0
    self.integralElbow = 0
    self.integralShoulder = 0

    self.lastShoulderError = 0
    self.lastElbowError = 0
    self.lastBaseError = 0

    self.pots = None
    self.target = None
    self.target_pots = None
	
  def pots_callback(self, msg):
    # 'Store' pot values
    self.pots = msg
    # rospy.loginfo("IN: targets: \n base: %f, elbow: %d, shoulder: %d" % (self.target_pots.baseVal, self.target_pots.elbowVal, self.target_pots.shoulderVal))
    rospy.loginfo("IN: basePos: %d, elbowPos: %d, shoulderPos: %d" % (self.pots.basePos, self.pots.elbowPos, self.pots.shoulderPos))
    try:
      output = self.get_vels()
    #try:
     # output = self.get_pot_vels()
    except Exception as e:
      rospy.loginfo("Exception: {} \n On line number: {}".format(e, sys.exc_info()[-1].tb_lineno))
      output = armCtrl()
      output.baseVel = output.shoulderVel = output.elbowVel = 0
      rospy.loginfo(e)
    rospy.loginfo("OUT: baseVel: %d, elbowVel: %d, shoulderVel: %d" % (output.baseVel, output.elbowVel, output.shoulderVel))
    pub.publish(output)	
  
  def target_callback(self, msg):
  # 'Store' target values
    try:
      self.target = msg
      rospy.loginfo("IN: targets: %d, %d, %d" % (self.target.x, self.target.y, self.target.z))
    except:
      rospy.loginfo("Error on target reciept")


# Takes in target pot values and publishes output velocities to get current pot values to match 
  def target_pots_callback(self, msg):
    try:
      self.target_pots = msg
      rospy.log(msg)      
#rospy.loginfo("TARGETS: base: %d, elbow: %d, shoulder: %d" %(self.target_pots.baseVal, self.target_pots.elbowVal, self.target_pots.shoulderVal))
    except Exception as e:
      rospy.loginfo("Exception: {} \n On line number: {}".format(e, sys.exc_info()[-1].tb_lineno))

  # Outputs desired velocities for all arm motors in the form of an armCtrl message.
  # Claw functionality can be easily added later. 
  #
  def get_vels(self):
    nowBase = self.pots.basePos
    nowShoulder = self.pots.shoulderPos
    nowElbow = self.pots.elbowPos       
    
    angles = [0,0,0]
    
    # replacing base solving with x = 0 so we can use x message for base pot val
    if kinematics.solve(float(0), float(self.target.y), float(self.target.z), angles):
     # rospy.loginfo("entered kinematics if statement")  
    # newBase = angles[0]*162.9 - offsetBase
      newShoulder = angles[1]*162.9 - offsetShoulder
      #rospy.loginfo("angles[1] reached")
      newElbow = angles[2]*162.9 - offsetElbow
      #rospy.loginfo("angles[2] reached")    
    # errorBase = newBase - nowBase
      errorBase = self.target.x - nowBase
      errorShoulder= newShoulder - nowShoulder
      errorElbow = newElbow - nowElbow
     
      rospy.loginfo("ERRORS: base: %d  elbow: %d shoulder: %d" % (errorBase, errorElbow, errorShoulder))

      msg = armCtrl()
      rospy.loginfo("TARGETS:  base: %d  elbow: %d  shoulder: %d" % (nowBase + errorBase, newElbow, newShoulder))

      setSpeed = 127
      kpBase = 0.25
      kpShoulder = 1      
      kpElbow = 2
      kiBase = 0
      kiShoulder = 0
      kiElbow = 0	
      kd = 0
 
      rospy.loginfo("Base Integral: %d" % self.integralBase)
 #     rospy.loginfo("Elbow Integral: %d" % self.integralElbow)
#      rospy.loginfo("Shoulder Integral: %d" % self.integralShoulder)

      # define acceptable deviation
      dev = 4
	
      #set velocities nonzero if outside permitted error
      if errorBase > dev:
        msg.baseVel = min(25, int(kpBase * errorBase + kiBase*self.integralBase))
      elif (-1*dev <= errorBase <= dev):
        self.integralBase = 0
        msg.baseVel = 0
      else: 
        msg.baseVel = max(-1*25, int(kpBase * errorBase + kiBase*self.integralBase))

      if errorElbow > dev:
         msg.elbowVel = min(setSpeed, int(kpElbow * errorElbow +kiElbow*self.integralElbow))
      elif (-1*dev <= errorElbow <= dev):
        self.integralElbow = 0
        msg.elbowVel = 0 
      else:
        msg.elbowVel = max(-1*setSpeed,int(kpElbow * errorElbow + kiElbow*self.integralElbow))

      if errorShoulder > dev:
        msg.shoulderVel = min(setSpeed, int(kpShoulder * errorShoulder + kiShoulder*self.integralShoulder))
      elif (-1*dev <= errorShoulder <= dev):
        self.integralShoudler = 0
        msg.shoulderVel = 0
      else:
        msg.shoulderVel = max(-1*setSpeed, int(kpShoulder * errorShoulder + kiShoulder*self.integralShoulder))

   # Security stops so arm doesn't destroy itself
      if not (sweepMinShoulder < self.pots.shoulderPos < sweepMaxShoulder):
        rospy.loginfo("WARNING: Shoulder near mechanical limit. Stopping motor.")
	msg.shoulderVel = 0
      if not (sweepMinElbow < self.pots.elbowPos < sweepMaxElbow):
	rospy.loginfo("WARNING: Elbow near mechanical limit. Stopping motor.")
	msg.elbowVel = 0	
      
      if (dev < abs(msg.elbowVel) < 50):
        self.integralElbow = self.integralElbow + (errorElbow * 0.02)
      if (dev < abs(errorShoulder) < 50):
        self.integralShoulder = self.integralShoulder + (errorShoulder * 0.02)
      if (dev < abs(errorBase) < 40):
        self.integralBase = self.integralBase + (errorBase * 0.02)

      if (msg.shoulderVel == 0):
	self.integralShoulder = 0

      if(msg.shoulderVel > 127):
	msg.shoulderVel = 127
      if(msg.shoulderVel < -127):
	msg.shoulderVel = -127
		
      if(abs(256 - offsetShoulder - self.pots.shoulderPos) > 15):
	self.integralShoulder = 0
      
      return msg
    msg = armCtrl()
    rospy.loginfo("ERROR: kinematics did not work")
    msg.baseVel = msg.shoulderVel = msg.elbowVel = 0
    return msg

  def get_pot_vels(self):
    try:
      nowBase = self.pots.basePos
      nowShoulder = self.pots.shoulderPos
      nowElbow = self.pots.elbowPos      
      setSpeed = 127
      kp = 2
      ki = 0

      errorBase = self.target_pots.baseVal - nowBase
      errorShoulder = self.target_pots.shoulderVal - nowShoulder
      errorElbow = self.target_pots.elbowVal - nowElbow
      
      self.integralElbow = self.integralElbow + (errorElbow * 0.02)
      self.integralShoulder = self.integralShoulder + (errorShoulder * 0.02)
      self.integralBase = self.integralBase + (errorBase * 0.02)

      #set velocities nonzero if outside permitted error
      if errorBase > 0:
        msg.baseVel = min(setSpeed, int(kp * errorBase + ki*self.integralBase))
      elif errorBase == 0:
        self.integralBase = 0
        msg.baseVel = 0
      else:
        msg.baseVel = max(-1*setSpeed, int(kp * errorBase + ki*self.integralBase))

      if errorElbow > 0:
        msg.elbowVel = min(setSpeed, int(kp * errorElbow + ki*self.integralElbow))
      elif errorElbow == 0:
        self.integralElbow = 0
        msg.elbowVel = 0
      else:
        msg.elbowVel = max(-1*setSpeed,int(kp * errorElbow + ki*self.integralElbow))

      if errorShoulder > 0:
        msg.shoulderVel = min(setSpeed, int(kp * errorShoulder + ki*self.integralShoulder))
      elif errorShoulder == 0:
        self.integralShoudler = 0
        msg.ShoulderVel = 0
      else:
        msg.shoulderVel = max(-1*setSpeed, int(kp * errorShoulder + ki*self.integralShoulder))

    # Security stops so arm doesn't destroy itself
      if not (sweepMinShoulder < self.pots.shoulderPos < sweepMaxShoulder):
        rospy.loginfo("WARNING: Shoulder near mechanical limit. Stopping motor.")
        msg.shoulderVel = 0
      if not (sweepMinElbow < self.pots.elbowPos < sweepMaxElbow):
        rospy.loginfo("WARNING: Elbow near mechanical limit. Stopping motor.")
        msg.elbowVel = 0
      return msg

    except Exception as e:
      msg = armCtrl()
      rospy.loginfo("Exception: {} \n On line number: {}".format(e, sys.exc_info()[-1].tb_lineno))
      # msg.baseVel = msg.elbowVel = msg.shoulderVel = 0
      return msg		

#def callback(data):
 # rospy.loginfo("IN: elbowPos: %d" % (data.elbowPos))
 # rospy.loginfo("IN: test: %d" % (data.basePos))
 # claw_output(data)

def control():
  rospy.init_node('arm_controller', anonymous=True)

  arm = Arm();

  rospy.Subscriber('claw_target', armTarget, arm.target_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('pot_targets', armPotTargets, arm.target_pots_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('arm_positions', armPos, arm.pots_callback, queue_size=1, tcp_nodelay=False)
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

