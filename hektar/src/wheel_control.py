#!/usr/bin/env python
import rospy
from hektar.msg import wheelVelocity 
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from hektar.cfg import HektarConfig


UPPER_LIMIT = 127
LOWER_LIMIT = -127


class Callback():
  def __init__(self):
    self.speed = 0
    self.variation_factor = 0.0
    self.offset_multiplier = 0.0
    self.offset_addition = 0
    self.pub = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)

  def wheel_callback(self, feedback):
    wheels = wheelVelocity()
    delta_L = 0
    delta_R = 0

    # scalar and addition offsets for left wheel to account for wheel speed
    # discrepancies
    computed_speedL = int((self.speed + (feedback.data * self.variation_factor)) \
      * self.offset_multiplier + self.offset_addition)
    if computed_speedL > UPPER_LIMIT:
      delta_L = computed_speedL - UPPER_LIMIT  
    
    computed_speedR = int(self.speed - (feedback.data * self.variation_factor))               
    if computed_speedR > UPPER_LIMIT:
      delta_R = computed_speedR - UPPER_LIMIT
    
    # difference between wheel speeds must be kept constant, so if one wheel goes above 
    # the max allowed speed value, this difference must be subtracted from the other wheel
    computed_speedR -= delta_L
    computed_speedL -= delta_R
    wheels.wheelL = max(LOWER_LIMIT, min(computed_speedL, UPPER_LIMIT))
    wheels.wheelR = max(LOWER_LIMIT, min(computed_speedR, UPPER_LIMIT))

    #rospy.loginfo(rospy.get_caller_id() + " Wheels: %f, %f", wheels.wheelL, wheels.wheelR)
    self.pub.publish(wheels)

  # required to dynamically reconfigure parameters
  def callback(self, config, level):
    rospy.loginfo("""Reconfigure Request: {speed}, {variation_factor}, \
      {offset_multiplier}, {offset_addition}""".format(**config))
    self.speed = config["speed"]
    self.variation_factor = config["variation_factor"]
    self.offset_multiplier = config["offset_multiplier"]
    self.offset_addition = config["offset_addition"]
    return config


def control():
  rospy.init_node('wheel_control', anonymous=True)
  
  callbacker = Callback()
  srv = Server(HektarConfig, callbacker.callback)
  rospy.Subscriber('control_effort', Float64, callbacker.wheel_callback, \
     queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

