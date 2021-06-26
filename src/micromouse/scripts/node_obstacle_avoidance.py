#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None


def clbk_laser(msg):
    regions = {
        # mapping laser data from 0 to 2
        'right':  msg.ranges[2]*10,
        'front':  msg.ranges[1]*10,
        'left':   msg.ranges[0]*10,
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > 1 and regions['left'] > 1 and regions['right'] > 1:
        state_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < 1 and regions['left'] > 1 and regions['right'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['left'] > 1 and regions['right'] < 1:
        state_description = 'case 3 - right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['left'] < 1 and regions['right'] > 1:
        state_description = 'case 4 - left'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['left'] > 1 and regions['right'] < 1:
        state_description = 'case 5 - front and right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 1 and regions['left'] < 1 and regions['right'] > 1:
        state_description = 'case 6 - front and left'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['left'] < 1 and regions['right'] < 1:
        state_description = 'case 7 - front and left and right'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['left'] < 1 and regions['right'] < 1:
        state_description = 'case 8 - left and right'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/micromouse/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/micromouse/laser/scan', LaserScan, clbk_laser)

    rospy.spin()


if __name__ == '__main__':
    main()
