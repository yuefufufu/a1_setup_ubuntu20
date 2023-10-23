#!/usr/bin/env python3

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import math
hoge = 0
n = 0

hz = 1.85



linx = 0.28 / hz
liny = 0
angz = 0

#angz = angz * math.pi / 180



linx = 0



if linx > 0.2 or liny > 0.2 or hz > 3.0:
    print("Input value error")
    sys.exit()

if linx == 0 and liny == 0 and angz == 0:
    flag = 0
else:
    flag = 1


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