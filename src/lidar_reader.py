#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(msg):
    ranges = msg.ranges

    # Güvenlik için sonsuz ve NaN değerleri temizle
    def clean(data):
        return [d for d in data if not math.isinf(d) and not math.isnan(d)]

    front = clean(ranges[0:5] + ranges[-5:])
    left  = clean(ranges[85:95])
    right = clean(ranges[265:275])

    if front:
        front_dist = sum(front) / len(front)
    else:
        front_dist = float('inf')

    if left:
        left_dist = sum(left) / len(left)
    else:
        left_dist = float('inf')

    if right:
        right_dist = sum(right) / len(right)
    else:
        right_dist = float('inf')

    rospy.loginfo(f"Ön: {front_dist:.2f} m | Sağ: {right_dist:.2f} m | Sol: {left_dist:.2f} m")

def main():
    rospy.init_node('lidar_reader')

    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.loginfo("LIDAR okunuyor...")
    rospy.spin()

if __name__ == '__main__':
    main()
