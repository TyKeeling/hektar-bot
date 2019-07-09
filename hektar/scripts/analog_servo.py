#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos
from rosserial_arduino.msg import Adc

pub = rospy.Publisher('servo', armCtrl, queue_size=10)

def callback(data):
    rospy.loginfo("received adc5: %d" % (data.adc5))
    claw_output()

def control():
   rospy.init_node('arm_controller', anonymous=True)
   rospy.Subscriber("adc", Adc, callback)
   rospy.spin()

def claw_output():
   publishee = armCtrl()
   publishee.l_claw = 120
   pub.publish(publishee)

if __name__ == '__main__':
    try:
      control()
    except rospy.ROSInterruptException: pass

