#!/usr/bin/env python


# Modified version of teleop_twist_keyboard.py (by ROS) 
# URL: https://github.com/ros-teleop/teleop_twist_keyboard
# For hektar, team 6's robot in the 2019 ENPH 253 robot competiton.

from __future__ import print_function
import rospy
import roslib
from hektar.msg import armPos, armCtrl, Claw
import kinematics
import math

import sys, select, termios, tty

armPub = rospy.Publisher('arm_commands', armCtrl, queue_size = 1)
clawPub = rospy.Publisher('grabber', Claw, queue_size = 1)

offsetShoulder = -224
offsetElbow = -296

settings = termios.tcgetattr(sys.stdin)

baseMsg = """
Reading from the keyboard  and publishing to arm_commands
---------------------------
Moving around:
    u    i
 h   j    k    l

anything else : stop
CTRL-C to quit
"""

moveBindings = {
	'h':(-1, 0, 0, 0, 0),
	'u':(0, 1, 0, 0, 0),
  'j':(0,-1, 0, 0, 0),
	'i':(0, 0, 1, 0, 0),
	'k':(0, 0, -1, 0, 0),
	'l':(1, 0, 0, 0, 0),
  'e':(0, 0, 0, 1, 0),
  'd':(0, 0, 0, -1, 0),
  'r':(0, 0, 0, 0, 1),
  'f':(0, 0, 0, 0, -1),
    }


class Servo:
  def __init__(self):
    self.angle = 0

  def getAngle(self):
    return self.angle

  def setAngle(self, angle):
    self.angle = angle


def location_callback(baseMsg):
  rospy.loginfo("Callback entered")
  speed = 40

  theta, r, z = kinematics.unsolve(0, math.pi - (baseMsg.shoulderPos + offsetShoulder)/162.9, -(baseMsg.elbowPos + offsetElbow)/162.9)
  rospy.loginfo("Location: theta: %d r: %d z: %d" % (base.getAngle(), r, z))
  angles = [0,0,0]
  kinematics.solve(float(0), float(r), float(z), angles)
  newShoulder = -(angles[1]-math.pi)*162.9 - offsetShoulder
  newElbow = (angles[2]*162.9) - offsetElbow
  rospy.loginfo("Pot vals: shoulder: %d, elbow:%d" % (newShoulder, newElbow))
  key = getKey()
  rospy.loginfo("Key: %s " % key)
  if key in moveBindings.keys():
       turn = moveBindings[key][0]
       elbow = moveBindings[key][1]
       shoulder = moveBindings[key][2]
       grabLeft = moveBindings[key][3]
       grabRight = moveBindings[key][4]
  else:
       turn = 0
       elbow = 0
       shoulder = 0
       grabLeft = 0
       grabRight = 0

  rospy.loginfo("Left: %d, Right: %d" % (grabLeft, grabRight))
  # handle claw positon
  clawMsg = Claw()
  if (grabLeft == 1):
    claw_l.setAngle(180)
  if (grabLeft == -1):
    claw_l.setAngle(0)
  if (grabRight == 1):
    claw_r.setAngle(180)
  if (grabRight == -1):
    claw_r.setAngle(0)


  rospy.loginfo("Claw commands: L: %d R: %d" % (claw_l.getAngle(), claw_r.getAngle()))

  clawMsg.posL = claw_l.getAngle()
  clawMsg.posR = claw_r.getAngle()
  clawPub.publish(clawMsg)


  # handle arm position
  baseMsg = armCtrl()
  baseMsg.elbowVel = elbow*speed
  baseMsg.shoulderVel = shoulder*speed
  baseMsg.baseVel = base.getAngle() + (5*turn)
  if (baseMsg.baseVel > 90):
    baseMsg.baseVel = 90
  elif (baseMsg.baseVel < -90):
    baseMsg.baseVel = -90

  base.setAngle(int(baseMsg.baseVel))
  armPub.publish(baseMsg)





def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)

    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
      base = Servo()
      base.setAngle(90)
      claw_l = Servo()
      claw_l.setAngle(180)
      claw_r = Servo()
      claw_r.setAngle(180)
      rospy.loginfo("Servos created")

      rospy.init_node('keyboard_arm', anonymous=True)
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      while not rospy.is_shutdown():
      	rospy.Subscriber('arm_positions', armPos, location_callback, queue_size=1, tcp_nodelay=False)

      rospy.spin()
