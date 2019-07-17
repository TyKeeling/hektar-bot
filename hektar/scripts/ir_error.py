#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Float64


# PARAMS and ASSUMPTIONS
#* Input: array of IR sensor vals (0-1024)
#* Output: error value (-2, 2) relative to setpoint
#
#* ASSUME setpoint is 0 (center)

pub = rospy.Publisher('state', Float64, queue_size=1)

# Initilaize parameters

def array_callback(msg):
  sensor0 = 1023 - msg.ir_0
  sensor1 = 1023 - msg.ir_1
  sensor2 = 1023 - msg.ir_2
  sensor3 = 1023 - msg.ir_3
  sensor4 = 1023 - msg.ir_4
  
  pos = Float64()
  pos.data = (sensor0 * -2 + sensor1 * -1 + sensor3 + sensor4 * 2) / ( 1.0 * (sensor1 + sensor1 + sensor2 + sensor3 + sensor4) )
  rospy.loginfo(rospy.get_caller_id() + " Error: %f", pos.data)
  pub.publish(pos)


def control():
  rospy.init_node('ir_error', anonymous=True)

  rospy.Subscriber('ir_array', IRarray, array_callback, queue_size=1, tcp_nodelay=False)   
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

