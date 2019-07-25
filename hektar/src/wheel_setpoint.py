#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def talker():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    pub = rospy.Publisher('setpoint', Float64, queue_size=10)
    rospy.init_node('setpoint_node', anonymous=True)
    setpoint = Float64()
    setpoint.data = 0
    pub.publish(setpoint)
    setpoint = Float64()
    setpoint.data = 0
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(setpoint)
        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

