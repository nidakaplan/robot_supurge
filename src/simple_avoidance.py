#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

def scan_callback(msg):
    ranges = msg.ranges

    def clean(data):
        return [d for d in data if not math.isinf(d) and not math.isnan(d)]

    # Ön mesafe (0 derece)
    front = clean(ranges[0:5] + ranges[-5:])

    if front:
        front_dist = sum(front) / len(front)
    else:
        front_dist = float('inf')

    cmd = Twist()

    if front_dist < 0.5:
        # Engel var → sola dön
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
    else:
        # Yol açık → ileri git
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0

    cmd_pub.publish(cmd)

def main():
    global cmd_pub

    rospy.init_node('simple_avoidance')

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
