#!/usr/bin/env python
import rospy
from hektar.msg import wheelVelocity 
from std_msgs.msg import Float64

speed = 70
variation_factor = 1.0

pub = rospy.Publisher('wheel_output', wheelVelocity, queue_size=1)

# Initilaize parameters

def wheel_callback(feedback):
  wheels = wheelVelocity()
  wheels.wheelL = speed + feedback.data * variation_factor 
  wheels.wheelR = speed - feedback.data * variation_factor
  rospy.loginfo(rospy.get_caller_id() + " Wheels: %d, %d", wheels.wheelL, wheels.wheelR)
  pub.publish(wheels)


def control():
  rospy.init_node('wheel_control', anonymous=True)

  rospy.Subscriber('control_effort', Float64, wheel_callback, queue_size=1, tcp_nodelay=False)   
 
  rospy.spin()

if __name__ == '__main__':
  try:
    control()
  except rospy.ROSInterruptException: pass

