#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from hektar.cfg import HektarConfig

# PARAMS and ASSUMPTIONS
#* Input: array of IR sensor vals (0-1024)
#* Output: error value (-2, 2) relative to setpoint
#
#* ASSUME setpoint is 0 (center)


OFF_TAPE_ERROR = 6


class Ir_Error():
  def __init__(self):
    self.pub = rospy.Publisher('state', Float64, queue_size=1)
    self.threshold = 0
    self.lastPos = Float64()
    self.lastPos.data = 0

  def array_callback(self, msg):
    sensors = [msg.ir_0, msg.ir_1, msg.ir_2, msg.ir_3, msg.ir_4]
    pos = Float64()

    if self.all_sensors_off_tape(sensors):
      if self.lastPos.data > 1.5: 
        pos.data = OFF_TAPE_ERROR

      elif self.lastPos.data < -1.5:
        pos.data = -OFF_TAPE_ERROR
	  
      else:
        pos.data = 0

    else:
        pos.data = -sensors.index(min(sensors)) + 2 

    rospy.loginfo(rospy.get_caller_id() + " Error: %f", pos.data)
    self.lastPos.data = pos.data
    self.pub.publish(pos)

  def all_sensors_off_tape(self, sensors):
    for sensor in sensors:
      if sensor < self.threshold:
        return False
    return True

  def callback(self, config, level):
    rospy.loginfo("""Reconfigure Request: {threshold}""".format(**config))
    self.threshold = config["threshold"]
    return config


def control():
  rospy.init_node('ir_error', anonymous=True)
  
  callbacker = Ir_Error()
  srv = Server(HektarConfig, callbacker.callback)
  rospy.Subscriber('ir_array', IRarray, callbacker.array_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

