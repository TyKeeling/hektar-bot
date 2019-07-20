#!/usr/bin/env python
import rospy
from hektar.msg import IRarray
from std_msgs.msg import Int8





pub = rospy.Publisher('ir_state', Int8, queue_size=1)
threshold = 600


def array_callback(msg):
  sensors = [msg.ir_0, msg.ir_1, msg.ir_2, msg.ir_3, msg.ir_4]
  path = Int8()

  
  for val in range(len(sensors)):
    if sensors[val] > threshold:
        sensors[val] = 0
    else:
        sensors[val] = 1

  if sensors == [1,0,1,0,0] or sensors == [0,1,0,1,0] \
    or sensors == [0,0,1,0,1] or sensors == [1, 0, 0, 1, 0] \
    or sensors == [0, 1, 0, 0, 1] or sensors == [1, 0, 0, 0, 1] \
    or sensors == [1, 1, 0, 1, 0] or sensors == [0, 1, 0, 1, 1]:

    path.data = 1
  elif sensors == [1,1,0,0,0] or sensors == [1,1,1,0,0] \
     or sensors == [0,0,1,1,1] or sensors == [0,0,0,1,1] \
     or sensors == [0, 1, 1, 1, 1, ] or sensors == [1, 1, 1, 1, 0]:
    path.data = 2
  else:
    path.data = 0
  

  rospy.loginfo(rospy.get_caller_id() + " ir_state mode: %d", path.data)
  pub.publish(path)


def control():
  rospy.init_node('ir_state', anonymous=True)

  rospy.Subscriber('ir_array', IRarray, array_callback, queue_size=1, tcp_nodelay=False)   
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

