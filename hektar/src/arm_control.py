#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos, armTarget, armPotTargets
import time
from math import pi
import kinematics

pub = rospy.Publisher('arm_commands', armCtrl, queue_size=10)

# Initilaize parameters
# Base measurements not yet taken
sweepMinBase = 0
sweepMaxBase = 512
angleMinBase = -pi/4
angleMaxBase = pi/4

sweepMinShoulder = 248 # 800mV output
sweepMaxShoulder = 621 # 2V output
angleMinShoulder = pi / 2
angleMaxShoulder = 5*pi/6

sweepMinElbow = 186 # 600mV output
sweepMaxElbow = 621  # 2V output
angleMinElbow = -pi/4
angleMaxElbow = pi/2

angleMinGripper = pi/2
angleMaxGripper = 0

class Arm:
  def __init__(self):

    self.pots = None
    self.target = None
    self.target_pots = None
	
  def pots_callback(self, msg):
    # 'Store' pot values
    self.pots = msg
    rospy.loginfo("IN: positions: \n base: %d, elbow: %d, shoulder: %d" % (self.pots.basePos, self.pots.elbowPos, self.pots.shoulderPos))
    # output = self.get_vels()
    output = self.get_pot_vels()
    rospy.loginfo("OUT: shoulderVel: %d" % output.shoulderVel)
    rospy.loginfo("OUT: elbowVel: %d" % output.elbowVel)
    rospy.loginfo("OUT: baseVel: %d" % output.baseVel)
    pub.publish(output)
    rospy.sleep(1)	
  
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
      rospy.loginfo("IN: target pot vals: \n base: %d, elbow: %d, shoulder: %d" %(self.target_pots.baseVal, self.target_pots.elbowVal, self.target_pots.shoudlerVal))
      output = self.get_pot_vels()
      rospy.loginfo("OUT: shoulderVel: %d" % output.shoulderVel)
      rospy.loginfo("OUT: elbowVel: %d" % output.elbowVel)
      rospy.loginfo("OUT: baseVel: %d" % output.baseVel)
      pub.publish(output)
      rospy.sleep(1)
    except:
      rospy.loginfo("Error on target pot receipt")

  # Outputs desired velocities for all arm motors in the form of an armCtrl message.
  # Claw functionality can be easily added later. 
  #
  def get_vels(self):
    try:
      nowBase = (self.pots.basePos) / 162.9
      nowShoulder = (self.pots.shoulderPos) / 162.9
      nowElbow = (self.pots.elbowPos) / 162.9
      rospy.loginfo("Angles: \n base: %f  elbow: %f  shoulder: %f" % (nowBase * 180/pi, nowElbow * 180/pi, nowShoulder * 180/pi))
      
      angles = [0,0,0]

      if kinematics.solve(float(self.target.x), float(self.target.y), float(self.target.z), angles):
        newBase = angles[0]
        newShoulder = angles[1]
        newElbow = angles[2]
        msg = armCtrl()
	rospy.loginfo("Target angles: \n base: %f  elbow: %f  shoulder: %f" % (newBase * 180/pi, newElbow * 180/pi, newShoulder * 180/pi))
        #set velocities nonzero if outside permitted error
        if newBase > nowBase:
	  msg.baseVel = min(127, 75 * (newBase - nowBase))
	else:
	  msg.baseVel = max(-127, 75 * (newBase - nowBase))
	
	if newElbow > nowElbow:
          msg.elbowVel = min(127, 75 * (newElbow - nowElbow))
        else:
          msg.elbowVel = max(-127, 75 * (newElbow - nowElbow))

	if newBase > nowBase:
          msg.shoulderVel = min(127, 75 * (newShoulder - nowShoulder))
        else:
          msg.shoulderVel = max(-127, 75 * (newShoulder - nowShoulder))

	# Security stops so arm doesn't destroy itself
#	if not (380 < self.pots.shoulderPos < 1015):
#	  rospy.loginfo("WARNING: Shoulder near mechanical limit. Stopping motor.")
#	  msg.shoulderVel = 0
#	if not (500 < self.pots.elbowPos < 800):
#	  rospy.loginfo("WARNING: Elbow near mechanical limit. Stopping motor.")
#	  msg.elbowVel = 0

	
    	return msg
	    
    except:
      msg = armCtrl()
      msg.baseVel = msg.elbowVel = msg.shoulderVel = 0
      return msg

  def get_pot_vels(self):
    try:
      nowBase = self.pots.basePos
      nowShoulder = self.pots.shoulderPos
      nowElbow = self.pots.elbowPos
      rospy.loginfo("Current: \n base: %f  elbow: %f  shoulder: %f" % (nowBase, nowElbow, nowShoulder))
 
      newBase = self.target_pots.baseVal 
      newShoulder = self.target_pots.shoulderVal
      newElbow = self.target_pots.elbowVal
      msg = armCtrl()
      rospy.loginfo("Targets: \n base: %f  elbow: %f  shoulder: %f" % (newBase, newElbow, newShoulder))
        
      kp = 3
      #set velocities nonzero if outside permitted error
      if newBase > nowBase:
        msg.baseVel = min(127, kp * (newBase - nowBase))
      else:
        msg.baseVel = max(-127, kp * (newBase - nowBase))

      if newElbow > nowElbow:
        msg.elbowVel = min(127, kp * (newElbow - nowElbow))
      else:
        msg.elbowVel = max(-127, kp * (newElbow - nowElbow))

      if newShoulder > nowShoulder:
        msg.shoulderVel = min(127, kp * (newShoulder - nowShoulder))
      else:
        msg.shoulderVel = max(-127, kp * (newShoulder - nowShoulder))

    # Security stops so arm doesn't destroy itself
    #       if not (380 < self.pots.shoulderPos < 1015):
    #         rospy.loginfo("WARNING: Shoulder near mechanical limit. Stopping motor.")
    #         msg.shoulderVel = 0
    #       if not (500 < self.pots.elbowPos < 800):
    #         rospy.loginfo("WARNING: Elbow near mechanical limit. Stopping motor.")
    #         msg.elbowVel = 0
      return msg

    except:
      msg = armCtrl()
      msg.baseVel = msg.elbowVel = msg.shoulderVel = 0
      return msg


		

#def callback(data):
 # rospy.loginfo("IN: elbowPos: %d" % (data.elbowPos))
 # rospy.loginfo("IN: test: %d" % (data.basePos))
 # claw_output(data)

def control():
  rospy.init_node('arm_controller', anonymous=True)

  arm = Arm();

  rospy.Subscriber('claw_target', armTarget, arm.target_callback, queue_size=1, tcp_nodelay=False)  
  rospy.Subscriber('arm_positions', armPos, arm.pots_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('pot_targets', armPotTargets, arm.target_pots_callback, queue_size=1, tcp_nodelay=False) 
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

