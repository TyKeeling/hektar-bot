#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from dynamic_reconfigure.server import Server
from hektar.cfg import pidConfig



class Setpoint():
  def __init__(self):
      self.point = 250
      self.setpoint = rospy.Publisher('setpoint', Float64, queue_size=1)


  def reconfigure(self, config, level):
    rospy.loginfo("""Reconfigure Request: {target_pot}""".format(**config))
    self.point = config["target_pot"]
    self.setpoint.publish(self.point)
    return config

def control():
  rospy.init_node('pid_setpoint', anonymous=True)

  setpoint = Setpoint()
  srv = Server(pidConfig, setpoint.reconfigure)

  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
