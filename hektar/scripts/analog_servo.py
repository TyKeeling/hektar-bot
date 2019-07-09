#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos
from rosserial_arduino.msg import Adc

pub = rospy.Publisher('servo', armCtrl, queue_size=10)

def callback(data):
    rospy.loginfo("received adc5: %d" % (data.adc5))
    claw_output(data.adc5)

def control():
   rospy.init_node('arm_controller', anonymous=True)
   rospy.Subscriber("adc", Adc, callback, queue_size=1, tcp_nodelay=False)
   rospy.spin()

def claw_output(adcVal):
   publishee = armCtrl()
   publishee.l_claw = (180*adcVal//1024)
   rospy.loginfo("l_claw: %d" % publishee.l_claw)
   pub.publish(publishee)

if __name__ == '__main__':
    try:
      control()
    except rospy.ROSInterruptException: pass

