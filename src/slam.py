#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

DESIRED_RIGHT = 0.3
DESIRED_FRONT = 0.6
FRONT_LIMIT = 0.4
DESIRED_FRONT_RIGHT = DESIRED_FRONT 
Kp = 0.9
Kd = 2.5
prev_error = 0.0
initialize = 1

def scan_callback(msg):
    global initialize
    global prev_error
    ranges = msg.ranges

    def clean(data):
        return [d for d in data if not math.isinf(d) and not math.isnan(d)]

    def average(data):
        return sum(data) / len(data) if data else float('inf')

    directions = {
        "front": clean(ranges[0:20] + ranges[340:360]),
        "front_right": clean(ranges[300:330]), 
        "right": clean(ranges[260:280]),
        "left" : clean(ranges[75:105]),
        "back" : clean(ranges[165:195])
    }

    distances = {}

    for name, data in directions.items():
        distances[name] = average(data)    

    front = distances["front"]
    front_right = distances["front_right"]
    right = distances["right"]
    left = distances["left"]
    back = distances["back"]

    cmd = Twist()

    if initialize:
        
        if front > DESIRED_FRONT or math.isinf(front):
            cmd.linear.x = 0.2 
            cmd.angular.z = 0.0
        else: 
            initialize = 0
        
        cmd_pub.publish(cmd)
        return

    error = DESIRED_RIGHT - right
    derivative = error - prev_error
    steering = (Kp * error) + (Kd * derivative)
    steering = max(min(steering, 0.6), -0.6)
    prev_error = error


    if front < DESIRED_FRONT:
        cmd.linear.x = 0.1
        cmd.angular.z = 0.5
        if front < FRONT_LIMIT:
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0 
    elif front_right < (DESIRED_FRONT_RIGHT):
        cmd.linear.x = 0.15
        cmd.angular.z = 0.5
    else:
        cmd.linear.x = 0.2
        cmd.angular.z = steering

    cmd_pub.publish(cmd)

def main():
    global cmd_pub
    rospy.init_node('slam')
    

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
