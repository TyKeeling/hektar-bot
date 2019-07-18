#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Float64


# PARAMS and ASSUMPTIONS
#* Input: array of IR sensor vals (0-1024)
#* Output: error value (-2, 2) relative to setpoint
#
#* ASSUME setpoint is 0 (center)

THRESHOLD = 500

pub = rospy.Publisher('state', Float64, queue_size=1)
lastPos = Float64()
lastPos.data = 0


def array_callback(msg):
  sensors = [msg.ir_0, msg.ir_1, msg.ir_2, msg.ir_3, msg.ir_4]
  pos = Float64()

  if above_threshold(sensors):
    if lastPos.data > 1.5: 
        pos.data = 6
    elif lastPos.data < -1.5:
        pos.data = -6
  else: pos.data = sensors.index(min(sensors)) -2 

  rospy.loginfo(rospy.get_caller_id() + " Error: %f", pos.data)
  lastPos.data = pos.data
  pub.publish(pos)

def above_threshold(sensors):
  for sensor in sensors:
    if sensor < THRESHOLD:
      return False
  return True

def control():
  rospy.init_node('ir_error', anonymous=True)

  rospy.Subscriber('ir_array', IRarray, array_callback, queue_size=1, tcp_nodelay=False)   
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

