#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, Int8
from hektar.msg import wheelVelocity
from hektar.cfg import HektarConfig

# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.

class Master():
  def __init__(self):
    self.enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    self.wheels = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
    self.speed  = rospy.Publisher("set_speed", Int8, queue_size=1)

    self.featuresHit = 0
    self.left = True

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
    stop = wheelVelocity()
    stop.wheelL = stop.wheelR = 0

    rospy.sleep(0.03) #solution to avoid wheel_control_output collisions
                      #there is some latency from PId to wheel control outptu
                      #which was leading to messages being sent after stop.
    self.wheels.publish(stop)
    rospy.sleep(0.5)
    motion = wheelVelocity()

    #if the tape triggers a stop then we will have to add a pass for no. 2
    if self.left:
      if   self.featuresHit == 0: 
        motion.wheelL = 0
        motion.wheelR = 30
        self.wheels.publish(motion)
        rospy.sleep(0.3) # random guess  
        self.wheels.publish(stop)
      elif self.featuresHit == 1: 
        motion.wheelL = 30
        motion.wheelR = 30
        self.wheels.publish(motion)
        rospy.sleep(0.3) 
        self.wheels.publish(stop)
        speedVal = Int8(data=40)
        self.speed.publish(speedVal)
      elif self.featuresHit == 2: 
        self.wheels.publish(stop)
        # come claw action: pickup (potd, potb, potbase)
        # but now I'm wondering if PID control should be used here 
        # so that the bot is further alligned.

    self.featuresHit = self.featuresHit + 1

    enabler.data = True
    self.enable.publish(enabler)

#  def config_callback(self, config, level):
#    return config


def control():
  rospy.init_node('control_master', anonymous=True)
 
  master = Master()

  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

