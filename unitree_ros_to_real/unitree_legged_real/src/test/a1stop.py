#!/usr/bin/env python3

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
from geometry_msgs.msg import Twist
import sys, select, termios, tty
hoge = 0
n = 0

hz = 2.0


linx = 0
liny = 0
angz = 0
flag = 0


def main():
    rospy.init_node("publisher")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    msg = Twist()

    msg.linear.x = float(linx)
    msg.linear.y = float(liny)
    msg.linear.z = float(hz)
    msg.angular.x = float(flag)
    msg.angular.y = float(hoge)
    msg.angular.z = float(angz)

    time.sleep(0.5)
    pub.publish(msg)
    rospy.loginfo("published\n" '{}'.format(msg))

if __name__ == "__main__":
    main()