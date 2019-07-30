#!/usr/bin/env python
import rospy
from hektar.msg import encoderPos, wheelVelocity, Move
from hektar.cfg import DeadReckonConfig
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool

MAX_SPEED = 127

class Dead_Reckon():
  def __init__(self):
    self.wheels_pub = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
    self.done_pub = rospy.Publisher("done_reckoning", Bool, queue_size=1)
    self.ticks_to_stop = None
    self.encoder_pos = None
    self.target_posR = None
    self.target_posL = None
    self.at_target = False
    self.dead_reckon = False
    self.right_at_target = False
    self.left_at_target = False

  def drive_callback(self, encoder_pos):
    wheels = wheelVelocity()
    if self.dead_reckon:
        if not self.at_target:
            self.encoder_pos = encoder_pos

            if ((target_pos - encoder_pos.wheelR) > self.ticks_to_stop):
                wheels.wheelR = MAX_SPEED
            else:
                self.right_at_target = True
            if ((target_posL - encoder_pos.wheelL) > self.ticks_to_stop):
	        wheels.wheelL = MAX_SPEED
	    else:
	        self.left_at_target = True
        
	    if self.right_at_target and self.left_at_target:
	        self.at_target = True

            self.wheel_pub.publish(wheels)

	else:
	    wheels.wheelR = 0
	    wheels.wheelL = 0
	    self.wheel_pub.publish(wheels)
	    self.done_pub.publish(doneReckoning())
	    dead_reckon = False
    
    else:
        pass

  def dynamic_callback(self, config, level):
    rospy.loginfo("""Reconfigure Request: {ticks_to_stop}""".format(**config))
    self.ticks_to_stop = config['ticks_to_stop']
    return config

  def move_callback(self, delta_ticks):
    self.target_posR = self.encoder_pos.wheelR + delta_ticks.wheelR
    self.target_posL = self.encoder_pos.wheelL + delta_ticks.wheelL
    self.at_target = False
    self.dead_reckon = True
    self.right_at_target = False
    self.right_at_target = False


def control():
  rospy.init_node("dead_reckon", anonymous=True)

  reckoner = Dead_Reckon()
  dynam_srv = Server(DeadReckonConfig, reckoner.dynamic_callback)
  rospy.Subscriber('encoder_pos', encoderPos, reckoner.drive_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('move', Move, reckoner.move_callback, queue_size=1, tcp_nodelay=False)

  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
