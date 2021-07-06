#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf import transformations

import numpy as np
from itertools import *
from operator import itemgetter
import random
import math


class SimbhaBot(object):
    def __init__(self):

        rospy.init_node('node_simbha_bot')
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.clbk_odom)
        self.sub_laser = rospy.Subscriber('scan', LaserScan, self.clbk_laser)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(20)

        # Other Variables
        self.threshold = 1.5  # Threshold value for laser scan
        self.lin_x = 0.0
        self.ang_z = 0.0
        self.Kp = 0.05
        self.position_ = Point()
        self.yaw_ = 0

        # Destination Point
        self.desired_position_ = Point()
        self.desired_position_.x = 7.0
        self.desired_position_.y = -1.0

        # precision
        self.precision = 0.1

    def clbk_laser(self, msg):
        range_angles = np.arange(len(msg.ranges))
        ranges = np.array(msg.ranges)  # create a numpy array of scan points
        # stores TRUE or FALSE in an array if value greater/less than threshold in range
        range_mask = (ranges > self.threshold)
        # add range_angles value greater than threshold to a list stored in ranges
        ranges = list(range_angles[range_mask])
        max_gap = 8
        gap_list = []
        # enumerate(ranges) create a tuple of (index, element) in ranges. The groupby groups similar items together
        for k, g in groupby(enumerate(ranges), lambda x: x[1] - x[0]):
            gap_list.append(list(map(itemgetter(1), g)))
        gap_list.sort(key=len)
        largest_gap = gap_list[-1]  # last element in gap_list
        min_angle, max_angle = largest_gap[0]*(
            (msg.angle_increment)*180/math.pi), largest_gap[-1]*((msg.angle_increment)*180/math.pi)
        average_gap = (max_angle - min_angle)/2

        turn_angle = min_angle + average_gap

        print(min_angle, max_angle)
        print(max_gap, average_gap, turn_angle)

        if average_gap < max_gap:
            self.ang_z = 0.5
        else:
            self.go_to_position()

    def clbk_odom(self, msg):
        # position
        self.position_ = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def go_to_position(self):

        k_linear = 0.125
        k_angular = 1.0
        # Calculate distance between current position and destination
        err_pos = math.sqrt((self.desired_position_.x - self.position_.x)
                            ** 2 + (self.desired_position_.y - self.position_.y)**2)
        lin_speed = k_linear * err_pos

        # Angle between starting position and destination in radians
        desired_yaw = math.atan2(self.desired_position_.y - self.position_.y,
                                 self.desired_position_.x - self.desired_position_.x)
        err_yaw = desired_yaw - self.yaw_
        ang_speed = k_angular * err_yaw
        self.lin_x = lin_speed
        self.ang_z = ang_speed

    def __del__(self):
        self.sub_laser.unregister()
        self.sub_odom.unregister()
        rospy.loginfo(
            '\033[94m' + "Object of class SimbhaBot Deleted." + '\033[0m')


def main():
    sb = SimbhaBot()
    while not rospy.is_shutdown():
        sb.go_to_position()
        pos = Twist()
        pos.linear.x = sb.lin_x
        pos.angular.z = sb.ang_z
        sb.pub_vel.publish(pos)
        sb.rate.sleep()


if __name__ == '__main__':
    main()
