#!/usr/bin/env python
import rospy
from hektar.msg import wheelVelocity 
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from hektar.cfg import HektarConfig


class Callback():
  def __init__(self):
    self.speed = 70
    self.variation_factor = 0.5

  def wheel_callback(self, feedback):
    wheels = wheelVelocity()
    wheels.wheelL = self.speed + (feedback.data * self.variation_factor) 
    wheels.wheelR = self.speed - (feedback.data * self.variation_factor)
    rospy.loginfo(rospy.get_caller_id() + " Wheels: %d, %d", wheels.wheelL, wheels.wheelR)
    
    pub = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
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

