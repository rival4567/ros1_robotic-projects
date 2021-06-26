#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Global Variables
sensor_l, sensor_c, sensor_r = 0, 0, 0
pub = 0


def clbk_laser(msg):
    global sensor_l, sensor_c, sensor_r

    # mapping laser data from 0 to 1
    sensor_l = msg.ranges[0]*5
    sensor_c = msg.ranges[1]*5
    sensor_r = msg.ranges[2]*5

    # print("l: {} \t c: {} \t r: {}".format(sensor_l, sensor_c, sensor_r))


def motion_go_straight():
    global pub
    msg = Twist()
    msg.linear.x = 0.5
    pub.publish(msg)


def motion_stop():
    global pub
    msg = Twist()
    msg.linear.x = 0.0
    pub.publish(msg)


def main():

    global sensor_l, sensor_c, sensor_r
    global pub

    msg = Twist()

    rospy.init_node('node_maze_runner')

    sub = rospy.Subscriber('/micromouse/laser/scan', LaserScan, clbk_laser)
    pub = rospy.Publisher('/micromouse/cmd_vel', Twist, queue_size=1)

    # pub.publish(msg)

    # rospy.spin()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print("l: {} \t c: {} \t r: {}".format(sensor_l, sensor_c, sensor_r))
        if math.isinf(sensor_c):
            motion_go_straight()
        else:
            motion_stop()

        rate.sleep()


if __name__ == '__main__':
    main()
