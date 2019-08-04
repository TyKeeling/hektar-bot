#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos
from std_msgs.msg import Float64

class Arm_split():
  def __init__(self):
    self.shoulder_state = rospy.Publisher('/shoulder/state', Float64, queue_size=1)
    self.elbow_state = rospy.Publisher('/elbow/state', Float64, queue_size=1)

    self.toSerial = rospy.publisher('arm_commands', armCtrl, queue_size=1)

    this.armVelocity = armCtrl(0,0,0)

  def armposCB(self, msg):
    self.shoulder_state.pub(msg.elbowPos)
    self.elbow_state.pub(msg.shoulderPos)
    self.toSerial.pub(armVelocity)

  def shoulderCB(self, msg):
      self.armVelocity.shoulderVel = msg.data

  def elbowCB(self, msg):
      self.armVelocity.elbowVel = msg.data


def control():
  rospy.init_node('arm_split', anonymous=True)

  arm = Arm_split()

  rospy.Subscriber('arm_positions', armPos, arm.armposCB, queue_size=1)
  rospy.Subscriber('/shoulder/control_effort', Float64, arm.shoulderCB, queue_size=1)
  rospy.Subscriber('/elbow/control_effort', Float64, arm.elbowCB, queue_size=1)

  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
