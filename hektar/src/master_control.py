#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool
from hektar.cfg import HektarConfig

# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.

class Master():
  def __init__(self):
    self.enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    #iself.move = rospy.Publisher('move', MoveWheels)
    #this should be a subscriber instead actually
    self.featuresHit = 0
    self.left = True

  # service call  
  # outoputs the  number of encoder ticks   
  # thinking that one wheel moving one tick is about 1.14 deg or 2.3 for both 
  # also 66 ticks makes one full revolution of the wheel. 
  def send_position(self, leftWheel, rightWheel): 
    rospy.loginfo("Feature %d identified. Sleeping.", self.featuresHit)
    rospy.sleep(2) 

  def fork_analysis_callback(self, msg):
    rospy.loginfo("fork analysis callback called")
    enabler = Bool()
    enabler.data = False
    self.enable.publish(enabler)
    #write 0 speed

    if self.left:
      if self.featuresHit == 0: 
        self.send_position(33, 33)
        self.send_position(0, 33)
      elif self.featuresHit == 1:
        self.send_position(33, 33)
        self.send_position(0, 66)
      elif self.featuresHit == 2: pass
      elif self.featuresHit == 3: pass 
      elif self.featuresHit == 4:
        self.send_position(11, 11)
        # come claw action: pickup (potd, potb, potbase)
        self.send_position(78, 0)
      elif self.featuresHit == 5: pass 
      elif self.featuresHit == 6: pass
      elif self.featuresHit == 7:
        self.send_position(33, 33)
        # come claw action: pickup (potd, potb, potbase)
        # but now I'm wondering if PID control should be used here 
        # so that the bot is further alligned.
    else: 
      pass
    self.featuresHit = self.featuresHit + 1

    enabler.data = True
    self.enable.publish(enabler)

#  def config_callback(self, config, level):
#    return config


def control():
  rospy.init_node('control_master', anonymous=True)
 
  master = Master()
#  srv = Server(HektarConfig, master.config_callback)

  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

