#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Float64, Bool
from dynamic_reconfigure.server import Server
from hektar.cfg import HektarConfig

# PARAMS and ASSUMPTIONS
#* Input: array of IR sensor vals (0-1024)
#* Output: error value (-2, 2) relative to setpoint
#
#* ASSUME setpoint is 0 (center)


OFF_TAPE_ERROR = 6
FEATURE_SIZE = 2


class Ir_Error():
  def __init__(self):
    self.state_pub = rospy.Publisher('state', Float64, queue_size=1)
    self.feature_pub = rospy.Publisher('line_feature', Bool, queue_size=1)
    self.threshold = 600
    self.lastPos = Float64()
    self.lastPos.data = 0
    self.feature_increment = 0
    self.feature_hit = [False] * FEATURE_SIZE
    self.sentFlag = False

  def array_callback(self, msg):
    sensors = (msg.ir_0, msg.ir_1, msg.ir_2, msg.ir_3, msg.ir_4)
    pos = Float64()
    send = Bool()
    send.data = True
    sensors_threshold = []

    for val in range(len(sensors)):
      if sensors[val] > threshold:
        sensors_threshold.append(0)
      else:
        sensors_threshold.append(1)
    
    if sensors_threshold == [1,0,1,0,0] or sensors_threshold == [0,1,0,1,0] \
      or sensors_threshold == [0,0,1,0,1] or sensors_threshold == [1, 0, 0, 1, 0] \
      or sensors_threshold == [0, 1, 0, 0, 1] or sensors_threshold == [1, 0, 0, 0, 1] \
      or sensors_threshold == [1, 1, 0, 1, 0] or sensors_threshold == [0, 1, 0, 1, 1]:
        self.feature_hit[self.feature_increment] = True
    
    elif sensors_threshold == [1,1,0,0,0] or sensors_threshold == [1,1,1,0,0] \
      or sensors_threshold == [0,0,1,1,1] or sensors_threshold == [0,0,0,1,1] \
      or sensors_threshold == [0, 1, 1, 1, 1, ] or sensors_threshold == [1, 1, 1, 1, 0]:
        self.feature_hit[self.feature_increment] = True
    
    else:
      self.feature_hit[self.feature_increment] = False
    
    # all(): built in python function that returns True if all elements in a list are True
    if all(self.feature_hit) and not self.sentFlag:
      self.feature_pub.publish(send)
      rospy.loginfo("Analysis: feature hit. ")
      self.sentFlag = True
    
    # any(): built in python function that returns True if any element in a list is True
    elif not any(self.feature_hit):
      self.sentFlag = False

    self.feature_increment = (self.feature_increment + 1) % FEATURE_SIZE

    if all(i >= self.threshold for i in sensors):
      if self.lastPos.data > 1.5: 
        pos.data = OFF_TAPE_ERROR

      elif self.lastPos.data < -1.5:
        pos.data = -OFF_TAPE_ERROR
	  
      else:
        pos.data = 0

    else:
        pos.data = -sensors.index(min(sensors)) + 2 

    #rospy.loginfo(rospy.get_caller_id() + " Error: %f", pos.data)
    self.lastPos.data = pos.data
    self.state_pub.publish(pos)

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

