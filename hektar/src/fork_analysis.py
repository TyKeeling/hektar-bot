#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Bool



pub = rospy.Publisher('line_feature', Bool, queue_size=1)
threshold = 600
i = 0
N = 7
boolist = [False] * N
sentFlag = False
send = Bool()
send.data = True

def array_callback(msg):
  sensors = [msg.ir_0, msg.ir_1, msg.ir_2, msg.ir_3, msg.ir_4]
  global i, boolist, sentFlag

  for val in range(len(sensors)):
    if sensors[val] > threshold:
        sensors[val] = 0
    else:
        sensors[val] = 1

  if sensors == [1,0,1,0,0] or sensors == [0,1,0,1,0] \
    or sensors == [0,0,1,0,1] or sensors == [1, 0, 0, 1, 0] \
    or sensors == [0, 1, 0, 0, 1] or sensors == [1, 0, 0, 0, 1] \
    or sensors == [1, 1, 0, 1, 0] or sensors == [0, 1, 0, 1, 1]:
    boolist[i] = True
  elif sensors == [1,1,0,0,0] or sensors == [1,1,1,0,0] \
     or sensors == [0,0,1,1,1] or sensors == [0,0,0,1,1] \
     or sensors == [0, 1, 1, 1, 1, ] or sensors == [1, 1, 1, 1, 0]:
    boolist[i] = True
  else:
    boolist[i] = False
  
  if allTrue(boolist) and not sentFlag:
    pub.publish(send)
    rospy.loginfo("Analysis: feature hit.")
    sentFlag = True
  elif allFalse(boolist):
    sentFlag = False

  i = (i+1) % N 

def allFalse(list):
  for el in list:
    if el == True:
      return False
  return True

def allTrue(list):
  for el in list:
    if el == False:
      return False
  return True

def control():
  rospy.init_node('fork_analysis', anonymous=True)

  rospy.Subscriber('ir_array', IRarray, array_callback, queue_size=1, tcp_nodelay=False)   
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

