#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from hektar.cfg import ClawSliderConfig
from hektar.msg import armTarget

class Claw_Slider():
  def __init__(self):
    self.theta = 0
    self.r = 0
    self.z = 0 
    self.pub = rospy.Publisher('claw_target', armTarget, queue_size=1)
  
  def claw_slider_callback(self, config, level):
    rospy.loginfo("""Reconfigure request: {claw_theta}, {claw_r}, {claw_z}""".format(**config))
    self.theta = config['claw_theta']
    self.r = config['claw_r']
    self.z = config['claw_z']
    
    msg = armTarget()
    msg.theta = self.theta
    msg.r = self.r
    msg.z = self.z
    self.pub.publish(msg)
    return config


def control():
  rospy.init_node('claw_slider_node', anonymous=True)
  callbacker = Claw_Slider()
  srv = Server(ClawSliderConfig, callbacker.claw_slider_callback)
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

