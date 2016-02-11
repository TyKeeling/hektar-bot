#!/usr/bin/env python
# license removed for brevity
import rospy
from rosserial_arduino.msg import Adc

def talker():
    pub = rospy.Publisher('adc', Adc, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        hello_str = Adc()
        hello_str.adc5 = i % 1024
        i += 10
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
