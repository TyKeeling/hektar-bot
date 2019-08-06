#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from hektar.cfg import BaseServoConfig


class Arm_split():
  def __init__(self):
    self.shoulder_state = rospy.Publisher('shoulder/state', Float64, queue_size=1)
    self.elbow_state = rospy.Publisher('/elbow/state', Float64, queue_size=1)

    self.toSerial = rospy.Publisher('arm_commands', armCtrl, queue_size=1)

    self.armVelocity = armCtrl(0,0,0)

  def armposCB(self, msg):
    self.shoulder_state.publish(msg.shoulderPos)
    self.elbow_state.publish(msg.elbowPos)
    self.toSerial.publish(self.armVelocity)

  def shoulderCB(self, msg):
      self.armVelocity.shoulderVel = msg.data

  def elbowCB(self, msg):
      self.armVelocity.elbowVel = msg.data

  def baseCB(self, msg):
      self.armVelocity.baseVel = msg.data

  def reconfigure(self, config, level):
    rospy.loginfo("""Reconfigure Request: {servo_position}""".format(**config))
    self.armVelocity.baseVel = config["servo_position"]
    return config

def control():
  rospy.init_node('arm_split', anonymous=True)

  arm = Arm_split()

  srv = Server(BaseServoConfig, arm.reconfigure)
  rospy.Subscriber('arm_positions', armPos, arm.armposCB, queue_size=1)
  rospy.Subscriber('/shoulder/control_effort', Float64, arm.shoulderCB, queue_size=1)
  rospy.Subscriber('/elbow/control_effort', Float64, arm.elbowCB, queue_size=1)
  rospy.Subscriber('/base_setpoint', Int8, arm.baseCB, queue_size=1)

  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
