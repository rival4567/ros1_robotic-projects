#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan


def clbk_laser(msg):
    # mapping from 0 to 1
    regions = [
        msg.ranges[0]*5,
        msg.ranges[1]*5,
        msg.ranges[2]*5,
    ]
    rospy.loginfo(regions)


def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/micromouse/laser/scan', LaserScan, clbk_laser)

    rospy.spin()


if __name__ == '__main__':
    main()
