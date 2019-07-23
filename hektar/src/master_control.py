#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool
from hektar.cfg import HektarConfig

# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.

class Master():
  def __init__(self):
    self.enable = rospy.Publisher('enable', Bool, queue_size=1)
    #iself.move = rospy.Publisher('move', MoveWheels)
    #this should be a subscriber instead actually
    featuresHit = 0
    left = True

  # service call  
  # outoputs the  number of encoder ticks   
  # thinking that one wheel moving one tick is about 1.14 deg or 2.3 for both 
  # also 66 ticks makes one full revolution of the wheel. 
  def send_position(self, leftWheel, rightWheel): 
    pass
  def fork_analysis_callback(self, msg):
    #send enable=False to PID
    if left:
      if featuresHit == 0: 
        send_position(33, 33)
        send_position(0, 33)
      elif featuresHit == 1:
        send_position(33, 33)
        send_position(0, 66)
      elif featuresHit == 2: pass
      elif featuresHit == 3: pass 
      elif featuresHit == 4:
        send_position(11, 11)
        # come claw action: pickup (potd, potb, potbase)
        send_position(78, 0)
      elif featuresHit == 5: pass 
      elif featuresHit == 6: pass
      elif featuresHit == 7:
        send_position(33, 33)
        # come claw action: pickup (potd, potb, potbase)
        # but now I'm wondering if PID control should be used here 
        # so that the bot is further alligned.
    else: 
      pass
    featuresHit = featuresHit + 1
    #send enable=True to PID

  def config_callback(self, config, level):
    return config


def control():
  rospy.init_node('control_master', anonymous=True)
 
  master = Master()
  srv = Server(HektarConfig, master.config_callback)

  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

