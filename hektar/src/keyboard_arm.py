#!/usr/bin/env python


# Modified version of teleop-keyboard.py (by ROS) for hektar-bot

from __future__ import print_function
import rospy
import roslib

from hektar.msg import armCtrl

import sys, select, termios, tty

pub = rospy.Publisher('arm_commands', armCtrl, queue_size = 10)

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to arm_commands
---------------------------
Moving around:
    u    i 
 h   j    k    l

anything else : stop
CTRL-C to quit
"""

moveBindings = {
	'h':(-1, 0, 0),        
	'u':(0, 1, 0),
        'j':(0,-1,0),
	'i':(0, 0, 1),
	'k':(0, 0, -1),
	'l':(1, 0, 0)
    }


class Servo:
  def __init__(self):
    self.angle = 90

  def getAngle(self):
    return self.angle

  def setAngle(self, angle):
    self.angle = angle

      
	
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def control():
    rospy.init_node('keyboard_arm', anonymous=True)

    servo = Servo()
    servo.setAngle(90)


    turn = 0
    elbow = 0
    shoulder = 0

    speed = 50

    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                turn = moveBindings[key][0]
                elbow = moveBindings[key][1]
                shoulder = moveBindings[key][2]
             
            else:
                turn = 0
                elbow = 0
                shoulder = 0
                if (key == '\x03'):
                    break
            msg = armCtrl()
            msg.elbowVel = elbow*speed
            msg.shoulderVel = shoulder*speed 
	    msg.baseVel = servo.getAngle() + (5*turn)
         
            servo.setAngle(int(msg.baseVel))
          
            pub.publish(msg)
    except Exception as e:
	rospy.loginfo(e)
  

if __name__=="__main__":
    try:
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      control()

    except rospy.ROSInterruptException: pass


