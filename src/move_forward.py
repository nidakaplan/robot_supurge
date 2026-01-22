#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('move_forward_node')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    cmd = Twist()
    cmd.linear.x = 0.1   # ileri hız
    cmd.angular.z = 0.0 # dönme yok

    rospy.loginfo("Robot ileri gidiyor...")

    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    main()
