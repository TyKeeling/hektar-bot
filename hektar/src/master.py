#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, Int8, Int32
from hektar.msg import wheelVelocity, armTarget, Claw
from hektar.cfg import HektarConfig
# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.

r = rospy.Rate(10)

DEFAULT = -1000
TICKS_REV = 240 #ish
ENCODER_ERROR = 60
TIMEOUT = 5
ENCODER_SPEED = 70

class Master():
  def __init__(self):
    self.pid_enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    self.wheels = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
    self.speed  = rospy.Publisher("set_speed", Int8, queue_size=20)
    self.claw = rospy.Publisher("grabber", Claw, queue_size = 1) # publish angle from 0-180 for claw open/close
    self.shoulder = rospy.Publisher("shoulder/setpoint", Float64, queue_size=1)
    self.elbow = rospy.Publisher("elbow/setpoint", Float64, queue_size=1)
    self.base = rospy.Publisher("base_setpoint", Int8, queue_size=1)

    self.featuresHit = 0
    self.left = False
    self.collided = False
    self.featureCallback = False

    self.encoder_left = 0
    self.encoder_right = 0
    self.begin_left = DEFAULT
    self.begin_right = DEFAULT

  # outputs the number of encoder ticks
  # thinking that one wheel moving one tick is about 1.14 deg or 2.3 for both
  # also 95*2.5 ticks makes one full revolution of the wheel.
  # assumes that PID is disabled.
  def send_position(self, leftWheel, rightWheel): #send revolution * TICKS_REV
    wheel = wheelVelocity();
    leftTarget = leftWheel + self.encoder_left
    rightTarget = rightWheel + right.encoder_right
    leftDone, rightDone = False
    i = 0

    while leftDone == False or rightDone == False:
        if leftTarget - leftWheel > ENCODER_ERROR:
            wheel.wheelL = ENCODER_SPEED
        elif leftWheel - leftTarget > ENCODER_ERROR:
            wheel.wheelL = - ENCODER_SPEED
        else:
            wheel.wheelL = 0
            leftDone = True

        if rightTarget - rightWheel > ENCODER_ERROR:
            wheel.wheelR = ENCODER_SPEED
        elif rightWheel - rightTarget > ENCODER_ERROR:
            wheel.wheelR = - ENCODER_SPEED
        else:
            wheel.wheelR = 0
            rightDone = True

        wheels.publish(wheel)

        i += 1

        if i > 1000:
            rospy.loginfo("Timeout")

        r.sleep()


  def collision_callback(self, msg):
    self.collided = True
    rospy.loginfo("Collision detected!")

    self.pid_enable.publish(False)
    rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
                      # there is some latency from Pid to wheel control output
                      # which was leading to messages being sent after stop
    self.wheels.publish(0,0)
    rospy.sleep(0.5)

    # now what - do we back up? turn around? keep driving? check we are still on tape?? spin until we find tape?
    # seems like the strat should be that we 180 if we hit someone after the beeline to go collect stones, but if we hit
    # someone during the beeline we just slightly change trajectory and continue, otherwise just wait and keep going?
    # for now, let's just turn in place.
    self.wheels.publish(40,-40)
    rospy.sleep(2) #random guess - is this enough time to turn significantly?

    self.wheels.publish(0,0)
    self.pid_enable.publish(True)
    self.collided = False

  def switch_callback(self, msg): # set switch and reset the featues hit
      self.left = msg.data
      self.featuresHit = 0
      if self.left:
          rospy.loginfo("switched to Left mode, reset featuresHit")
      else:
          rospy.loginfo("switched to Right mode, reset featuresHit")


  #Thinking that if we get hit with a robot at a fork, we should continue on.
  #For now colisions are only for the IR array
  def fork_analysis_callback(self, msg):
    self.featureCallback = True
    #if self.collided: return
    rospy.loginfo("Feature %d identified. Sleeping.", self.featuresHit)
    self.pid_enable.publish(False)
    #write 0 speed
    stop = wheelVelocity()
    stop.wheelL = stop.wheelR = 0
    rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
                      # there is some latency from Pid to wheel control output
                      # which was leading to messages being sent after stop
    self.wheels.publish(stop)
    rospy.sleep(0.3)

    # RIGHT SIDE of the course:
    if not self.left or True:
      if self.featuresHit == 0:
        self.wheels.publish(-10, 50) # guesses for the left turn
        rospy.sleep(2.0)             # replace with encoders when ready
        self.wheels.publish(stop)

      elif self.featuresHit == 1:

        self.wheels.publish(-10, 50)
        rospy.sleep(1.5)
        self.wheels.publish(stop)
        self.speed.publish(60) #slow down once we have entered the higher circle

      elif self.featuresHit == 2: # First T: pickup stone
        self.wheels.publish(stop)
        rospy.loginfo("at the T intersection. Robot will be stopped until mode switch is changed.")
        rospy.sleep(30)

    # BEGIN: Sequence for Stone Pickup
        # x = 250
        # self.base.publish(-90)
        # self.shoulder.publish(x)
        # self.elbow.publish(250)
        # rospy.sleep(0.5)
        # while not self.claw_limit_switch or x < 400:
        #     x += 5
        #     self.shoulder.publish(x)
        #     rospy.sleep(0.05)
        # self.claw.publish(180,180) # close
        # rospy.sleep(0.1)
        # self.elbow.publish(150)  # lift up
        # rospy.sleep(0.2)
        # self.shoulder.publish(400) #return to resting position
        # self.elbow.publish(250)
    # END: Sequence for Stone Pickup

    else: #Left side of the course
      if self.featuresHit == 0:
        self.wheels.publish(50, -10) # guesses for the left turn
        rospy.sleep(2.0)             # replace with encoders when ready
        self.wheels.publish(stop)

      elif self.featuresHit == 1:

        self.wheels.publish(50, -10)
        rospy.sleep(1.5)
        self.wheels.publish(stop)
        self.speed.publish(60) #slow down once we have entered the higher circle

      elif self.featuresHit == 2: # First T: pickup stone
        self.wheels.publish(stop)
        rospy.loginfo("at the T intersection. Robot will be stopped until mode switch is changed.")
        rospy.sleep(30)

    # BEGIN: Sequence for Stone Pickup
        # x = 250
        # self.base.publish(-90)
        # self.shoulder.publish(x)
        # self.elbow.publish(250)
        # rospy.sleep(0.5)
        # while not self.claw_limit_switch or x < 400:
        #     x += 5
        #     self.shoulder.publish(x)
        #     rospy.sleep(0.05)
        # self.claw.publish(180,180) # close
        # rospy.sleep(0.1)
        # self.elbow.publish(150)  # lift up
        # rospy.sleep(0.2)
        # self.shoulder.publish(400) #return to resting position
        # self.elbow.publish(250)
    # END: Sequence for Stone Pickup


    self.featuresHit = self.featuresHit + 1
    self.pid_enable.publish(True)
    self.featureCallback=False

  def encoder_left_callback(self, msg): # set switch and reset the featues hit
      if self.begin_left == DEFAULT:
        self.begin_left = msg.data
      else:
        self.encoder_left = msg.data - self.begin_left

  def encoder_right_callback(self, msg): # set switch and reset the featues hit
      if self.begin_right == DEFAULT:
        self.begin_right = msg.data
      else:
        self.encoder_right = msg.data - self.begin_right

  def refresh(self):
      if not self.featureCallback:
        rospy.loginfo("Left Encoder: %d, Right Encoder: %d", self.encoder_left, self.encoder_right)
        if 2400 - self.encoder_right + 1600 - self.encoder_left < 300:
          self.pid_enable.publish(False)
          rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
          self.wheels.publish(0,0)
          rospy.loginfo("Stopping! Dead Reckon")

  def  cleanup():
    rospy.sleep(0.03)
    self.pid_enable.publish(True)

def control():
  rospy.init_node('control_master', anonymous=True)

  master = Master()

  rospy.Subscriber('collision', Bool, master.collision_callback, queue_size=1)
  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('left', Bool, master.switch_callback)
  rospy.Subscriber('encoder_left', Int32, master.encoder_left_callback, queue_size=1)
  rospy.Subscriber('encoder_right', Int32, master.encoder_right_callback, queue_size=1)

  rospy.on_shutdown(master.cleanup)


  while not rospy.is_shutdown() or True:
    master.refresh()
    r.sleep()
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
