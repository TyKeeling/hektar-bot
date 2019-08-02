#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, Int8
from hektar.msg import wheelVelocity, armTarget, Claw
from hektar.cfg import HektarConfig
from arm_targets import Target
# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.

class Master():
  def __init__(self):
    self.enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    self.wheels = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
    self.speed  = rospy.Publisher("set_speed", Int8, queue_size=1)
    self.claw = rospy.Publisher("grabber", Claw, queue_size = 1) # publish angle from 0-180 for claw open/close
    self.arm = rospy.Publisher("claw_target", armTarget, queue_size=1) # message in cylindrical coordinates
    self.speed.publish(99)
    

    self.featuresHit = 0
    self.left = True
    self.collided = False

  # outputs the number of encoder ticks   
  # thinking that one wheel moving one tick is about 1.14 deg or 2.3 for both 
  # also 66 ticks makes one full revolution of the wheel. 
  def send_position(self, leftWheel, rightWheel): 
    rospy.loginfo("Feature %d identified. Sleeping.", self.featuresHit)
    rospy.sleep(2) 

  def collision_callback(self, msg):
    self.collided = True
    rospy.loginfo("Collision detected!")

    enabler = Bool()
    enabler.data = False
    self.enable.publish(enabler)
    #write 0 speed
    stop = wheelVelocity()
    stop.wheelL = stop.wheelR = 0
    rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
                      # there is some latency from Pid to wheel control output 
                      # which was leading to messages being sent after stop
    self.wheels.publish(stop)
    rospy.sleep(0.5)

    motion = wheelVelocity()

    # now what - do we back up? turn around? keep driving? check we are still on tape?? spin until we find tape?
    # seems like the strat should be that we 180 if we hit someone after the beeline to go collect stones, but if we hit 
    # someone during the beeline we just slightly change trajectory and continue, otherwise just wait and keep going?

    # for now, let's just turn in place.
    motion.wheelL = 30
    motion.wheelR = -30
    self.wheels.publish(motion)
    rospy.sleep(1) #random guess - is this enough time to turn significantly?
    
    self.wheels.publish(stop)
    
    enabler.data = True
    self.enable.publish(enabler)

    self.collided = False

  def fork_analysis_callback(self, msg):
    if self.collided: return

    rospy.loginfo("fork analysis callback called")
    enabler = Bool()
    enabler.data = False
    self.enable.publish(enabler)
    #write 0 speed
    stop = wheelVelocity()
    stop.wheelL = stop.wheelR = 0
    rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
                      # there is some latency from Pid to wheel control output 
                      # which was leading to messages being sent after stop
    
    if self.collided: return

    self.wheels.publish(stop)
    rospy.sleep(0.5)

    if self.collided: return

    rospy.loginfo("Feature %d identified. Sleeping.", self.featuresHit)

    #if the tape triggers a stop then we will have to add a pass for no. 2
    if self.left:
      if self.featuresHit == 0 or True: 
        self.wheels.publish(0, 30)
        rospy.sleep(2.3) # random guess  
        self.wheels.publish(stop)
      elif self.featuresHit == 1: 
        self.wheels.publish(20, 40)
        rospy.sleep(2.3) 
        #if self.collided: return
	      #self.wheels.publish(stop)
        self.speed.publish(65)
      elif self.featuresHit == 2: 
        self.wheels.publish(stop)
        # open left claw
        leftGrip = Claw()
        leftGrip.posL = 180
        # align arm
        target = armTarget()
        target.theta = -90
        target.r = 250
        target.z = 100
        self.arm.publish(target)
        rospy.sleep(3)
        #lower arm
        target.z = 80
        self.arm.publish(target)
        rospy.sleep(1)
        # close left claw
        self.claw.publish(leftGrip)
        rospy.sleep(1)
        # lift stone away
        target.z = 120
        self.arm.publish(target)

        # but now I'm wondering if PID control should be used here 
        # so that the bot is further aligned.
    
    self.featuresHit = self.featuresHit + 1

    enabler.data = True
    self.enable.publish(enabler)

#  def config_callback(self, config, level):
#    return config


def control():
  rospy.init_node('control_master', anonymous=True)
 
  master = Master()

  rospy.Subscriber('collision', Bool, master.collision_callback, queue_size=1)
  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)   
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

