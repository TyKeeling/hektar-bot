#!/usr/bin/env python
import rospy
import math
from hektar.msg import wheelVelocity 
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from hektar.cfg import HektarConfig

pub = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)

class Callback():
  def __init__(self):
    self.speed = 0
    self.variation_factor = 0.0

  def wheel_callback(self, feedback):
    wheels = wheelVelocity()
    a = self.speed + int(feedback.data * self.variation_factor) 
    if a > 127:
      a = 127
    elif a < -127:
      a = -127
    wheels.wheelL = a
    
    a = self.speed - int(feedback.data * self.variation_factor)                
    if a > 127:
      a = 127
    elif a < -127:
      a = -127
    wheels.wheelR = a

    rospy.loginfo(rospy.get_caller_id() + " Wheels: %f, %f", wheels.wheelL, wheels.wheelR)
    
    pub.publish(wheels)

  # required to dynamically reconfigure parameters
  def callback(self, config, level):
    rospy.loginfo("""Reconfigure Request: {speed}, {variation_factor}""".format(**config))
    self.speed = config["speed"]
    self.variation_factor = config["variation_factor"]
    return config


def control():
  rospy.init_node('wheel_control', anonymous=True)
  
  callbacker = Callback()
  srv = Server(HektarConfig, callbacker.callback)
  rospy.Subscriber('control_effort', Float64, callbacker.wheel_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

