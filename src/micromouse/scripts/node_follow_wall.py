#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None
regions_ = {
    'right': 0,
    'front': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    global regions_
    regions_ = {
        # mapping from 0 to 2
        'right':  msg.ranges[2]*10,
        'front':  msg.ranges[1]*10,
        'left':   msg.ranges[0]*10,
    }

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 1.0

    if regions['front'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 3 - right'
        change_state(2)
    elif regions['front'] > d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 4 - left'
        change_state(0)
    elif regions['front'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'case 5 - front and right'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'case 6 - front and left'
        change_state(1)
    elif regions['front'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 7 - front and left and right'
        change_state(1)
    elif regions['front'] > d and regions['left'] < d and regions['right'] < d:
        state_description = 'case 8 - left and right'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.6
    msg.angular.z = -0.5
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.5
    return msg


def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 1.0
    return msg


def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/micromouse/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/micromouse/laser/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
