#!/usr/bin/env python
import rospy
from hektar.msg import armCtrl, armPos

pub = rospy.Publisher('arm_commands', armCtrl, queue_size=10)

def callback(data):
    rospy.loginfo("IN: elbowPos: %d" % (data.elbowPos))
    rospy.loginfo("IN: test: %d" % (data.basePos))
    claw_output(data)

def control():
   rospy.init_node('arm_controller', anonymous=True)
   rospy.Subscriber("arm_positions", armPos, callback, queue_size=1, tcp_nodelay=False)
   rospy.spin()

def claw_output(armPos):
   output = armCtrl()
   output.l_claw = (180*armPos.elbowPos//1024)
   output.elbowVel = (armPos.elbowPos - 512)/4
   rospy.loginfo("OUT: l_claw: %d" % output.l_claw)
   rospy.loginfo("OUT: elbowVel: %d" % output.elbowVel)
   pub.publish(output)
   #rospy.sleep(1)

if __name__ == '__main__':
    try:
      control()
    except rospy.ROSInterruptException: pass

