#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool, Int8, Int32, Float64
from hektar.msg import wheelVelocity, armTarget, Claw
from hektar.cfg import HektarConfig
# Master control header. This node takes the state of features in the course and dictates arm and wheel motion.


DEFAULT = -1000
TICKS_REV = 240 #ish
ENCODER_ERROR = 20
ENCODER_SPEED = 70
SHOULDER_INDEX = 0
ELBOW_INDEX = 1
BASE_INDEX = 2
LEFT_FIRST_STONE = 0
LEFT_SECOND_STONE = 1
RIGHT_FIRST_STONE = 2
RIGHT_SECOND_STONE = 3
POSTS = (
  (250, 250, 90),
  (250, 250, 90),
  (250, 250, -90),
  (250, 250, -90),
)
GAUNTLET = (250, 250)


class Master():
  def __init__(self):
    self.pid_enable = rospy.Publisher('pid_enable', Bool, queue_size=1)
    self.wheels = rospy.Publisher("wheel_output", wheelVelocity, queue_size=1)
    self.speed  = rospy.Publisher("set_speed", Int8, queue_size=1)
    self.claw = rospy.Publisher("grabber", Claw, queue_size = 1) # publish angle from 0-180 for claw open/close
    self.shoulder = rospy.Publisher("shoulder/setpoint", Float64, queue_size=1)
    self.elbow = rospy.Publisher("elbow/setpoint", Float64, queue_size=1)
    self.base = rospy.Publisher("base_setpoint", Int8, queue_size=1)

    self.featuresHit = 0
    self.claw_left = False
    self.claw_right = True
    self.left = False
    self.collided = False
    self.featureCallback = False

    self.encoder_left = 0
    self.encoder_right = 0
    self.begin_left = 0
    self.begin_right = 0

    self.posts = (
	  (250, 250, 90),
	  (250, 250, 90),
      (250, 250, -90),
	  (250, 250, -90),
	)

    self.pid_enable.publish(True)
    self.speed.publish(100)

  # outputs the number of encoder ticks
  # thinking that one wheel moving one tick is about 1.14 deg or 2.3 for both
  # also 95*2.5 ticks makes one full revolution of the wheel.
  # assumes that PID is disabled.
  def send_position(self, leftWheel, rightWheel): #send revolution * TICKS_REV
    wheel = wheelVelocity();
    leftTarget = leftWheel + self.encoder_left
    rightTarget = rightWheel + self.encoder_right
    leftDone, rightDone = False, False
    i = 0

    while leftDone == False or rightDone == False:
        if leftTarget - self.encoder_left > ENCODER_ERROR:
            wheel.wheelL = ENCODER_SPEED
        elif self.encoder_left - leftTarget > ENCODER_ERROR:
            wheel.wheelL = - ENCODER_SPEED
        else:
            wheel.wheelL = 0
            leftDone = True

        if rightTarget - self.encoder_right > ENCODER_ERROR:
            wheel.wheelR = ENCODER_SPEED
        elif self.encoder_right - rightTarget > ENCODER_ERROR:
            wheel.wheelR = - ENCODER_SPEED
        else:
            wheel.wheelR = 0
            rightDone = True

        self.wheels.publish(wheel)

        i += 1

        if i > 2000:
            rospy.loginfo("Timeout")
            break

        rospy.sleep(0.02)
    rospy.loginfo("Done Reckon")


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
      self.pid_enable.publish(True)
      self.featuresHit = 0
      self.begin_right = self.encoder_right
      self.begin_left  = self.encoder_left
      self.speed.publish(100)

      if self.left:
          rospy.loginfo("switched to Left mode, reset featuresHit and Encoders")
      else:
          rospy.loginfo("switched to Right mode, reset featuresHit and Encoders")

  def claw_callback_l(self, msg):
    self.claw_left = msg.data

  def claw_callback_r(self, msg):
    self.claw_right = msg.data

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
    if not self.left:
      if self.featuresHit == 0:
        self.wheels.publish(-10, 70) # guesses for the left turn
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
        rospy.sleep(10)
        #self.pickup_stone(RIGHT_FIRST_STONE)

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
		#self.pickup_stone(LEFT_FIRST_STONE)
        # U-TURN AND TAPE FOLLOW BACK TO FORK TO PLACE STONE (or continue on to get another stone)

    self.featuresHit = self.featuresHit + 1
    self.pid_enable.publish(True)
    self.featureCallback=False

  def encoder_left_callback(self, msg): # set switch and reset the featues hit
    self.encoder_left = msg.data - self.begin_left

  def encoder_right_callback(self, msg): # set switch and reset the featues hit
    self.encoder_right = msg.data - self.begin_right

  def pickup_stone(self, post_num):
    self.base.publish(POSTS[post_num][BASE_INDEX])
    self.shoulder.publish(POSTS[post_num][SHOULDER_INDEX])
    self.elbow.publish(POSTS[post_num][ELBOW_INDEX])
    rospy.sleep(2)
    i = POSTS[post_num][SHOULDER_INDEX]
    while not self.claw_limit_switch or i < 400:
      i += 5
      self.shoulder.publish(i)
      rospy.sleep(0.05)
    self.claw.publish(180, 180) #close
    rospy.sleep(0.1)
    self.elbow.publish(POSTS[post_num][ELBOW_INDEX] + LIFT_UP_OFFSET) # lift up
    rospy.sleep(0.2)
    self.shoulder.publish(POSTS[post_num][SHOULDER_INDEX]) # put shoulder back to start position
    self.elbow.publish(POSTS[post_num][ELBOW_INDEX]) # put elbow back to start position

	# take shoulder and elbow values now
    
  def place_stone(self):
    self.base.publish(0)
    self.elbow.publish(GAUNTLET[ELBOW_INDEX])
    self.shoulder.publish(GAUNTLET[SHOULDER_INDEX])
    rospy.sleep(2)
    self.claw.publish(0, 0) #open
    rospy.sleep(0.5)
    self.elbow.publish(GAUNTLET[ELBOW_INDEX])

  def refresh(self):
      if not self.featureCallback:
        rospy.loginfo("Left Encoder: %d, Right Encoder: %d", self.encoder_left, self.encoder_right)
        if 1600 - self.encoder_left < 200:
          self.pid_enable.publish(False)
          rospy.sleep(0.03) # solution to avoid wheel_control_output collisions
          self.wheels.publish(0,0)
          rospy.loginfo("Stopping! Dead Reckon")
          rospy.sleep(2)
          self.pid_enable.publish(True)

  def cleanup(self):
    rospy.sleep(0.03)
    self.pid_enable.publish(True)

def control():
  rospy.init_node('control_master', anonymous=True)
  r = rospy.Rate(10)

  master = Master()

  rospy.Subscriber('collision', Bool, master.collision_callback, queue_size=1)
  rospy.Subscriber('line_feature', Bool, master.fork_analysis_callback, queue_size=1, tcp_nodelay=False)
  rospy.Subscriber('left_side', Bool, master.switch_callback)
  rospy.Subscriber('limit_left', Bool, master.claw_callback_l)
  rospy.Subscriber('limit_right', Bool, master.claw_callback_r)
  rospy.Subscriber('encoder_left', Int32, master.encoder_left_callback, queue_size=1)
  rospy.Subscriber('encoder_right', Int32, master.encoder_right_callback, queue_size=1)

  rospy.on_shutdown(master.cleanup)


  #while not rospy.is_shutdown() or True:
 # rospy.sleep(2)
 # master.pid_enable.publish(False)
 # master.send_position(-320, 320)
 # master.wheels.publish(0,0)
 # master.pid_enable.publish(True)
  rospy.spin()


if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass
