#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Float64, Bool
from dynamic_reconfigure.server import Server
from hektar.cfg import pid



class Setpoint():
  def __init__(self):
      self.setpoint = 250
      self.setpoint = rospy.Publisher('setpoint', Float64, queue_size=1)


  def reconfigure(self, config, level):
    rospy.loginfo("""Reconfigure Request: {threshold}""".format(**config))
    self.setpoint = config["threshold"]
    setpoint.publish(setpoint)
    return config

def control():
  rospy.init_node('pid_setpoint', anonymous=True)

  setpoint = Setpoint()
  srv = Server(pid, setpoint.reconfigure)

  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
