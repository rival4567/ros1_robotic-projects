#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

import math


class MazeSolver:

    def __init__(self):

        rospy.init_node('node_maze_solver', anonymous=True)

        self.pub_vel = rospy.Publisher(
            '/micromouse/cmd_vel', Twist, queue_size=1)

        self.sub_odom = rospy.Subscriber(
            '/micromouse/odom', Odometry, self.clbk_odom)

        self.sub_laser = rospy.Subscriber(
            '/micromouse/laser/scan', LaserScan, self.clbk_laser)

        self.position_ = Point()
        self.yaw_ = 0
        self.sensor_ = []
        self.rate = rospy.Rate(20)  # Rate in Hz

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
        # fixing joint pos by subtracting 90 degrees because they're different in gazebo and ROS
        self.yaw_ = euler[2]-math.pi/2

    def clbk_laser(self, msg):
        self.sensor_ = {
            # mapping laser data from 0 to 2
            'right':  msg.ranges[2]*10,
            'front':  msg.ranges[1]*10,
            'left':   msg.ranges[0]*10,
        }

    def calc_distance(self, x, y):
        return math.sqrt(x**2 + y**2)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub_vel.publish(twist_msg)
        rospy.loginfo("reached centre of maze!")

    def move_bot(self):
        twist_msg = Twist()
        linear_x = 0
        angular_z = 0

        if self.sensor_['front'] > 1 and self.sensor_['left'] > 1 and self.sensor_['right'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0
        elif self.sensor_['front'] < 1 and self.sensor_['left'] > 1 and self.sensor_['right'] > 1:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = 0.3
        elif self.sensor_['front'] > 1 and self.sensor_['left'] > 1 and self.sensor_['right'] < 1:
            state_description = 'case 3 - right'
            linear_x = 0
            angular_z = 0.3
        elif self.sensor_['front'] > 1 and self.sensor_['left'] < 1 and self.sensor_['right'] > 1:
            state_description = 'case 4 - left'
            linear_x = 0
            angular_z = -0.3
        elif self.sensor_['front'] < 1 and self.sensor_['left'] > 1 and self.sensor_['right'] < 1:
            state_description = 'case 5 - front and right'
            linear_x = 0
            angular_z = 0.3
        elif self.sensor_['front'] < 1 and self.sensor_['left'] < 1 and self.sensor_['right'] > 1:
            state_description = 'case 6 - front and left'
            linear_x = 0
            angular_z = -0.3
        elif self.sensor_['front'] < 1 and self.sensor_['left'] < 1 and self.sensor_['right'] < 1:
            state_description = 'case 7 - front and left and right'
            linear_x = 0
            angular_z = 0.3
        elif self.sensor_['front'] > 1 and self.sensor_['left'] < 1 and self.sensor_['right'] < 1:
            state_description = 'case 8 - left and right'
            linear_x = 0.3
            angular_z = 0
        else:
            state_description = 'unknown case'
            rospy.loginfo(self.sensor_)

        rospy.loginfo(state_description)
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.pub_vel.publish(twist_msg)

    def solve_maze(self):
        # Infinite loop
        while not rospy.is_shutdown():
            if self.calc_distance(self.position_.x, self.position_.y) < 0.02:
                self.done()
            else:
                self.move_bot()
            self.rate.sleep()

    def __del__(self):
        self.sub_odom.unregister()
        self.sub_laser.unregister()
        rospy.loginfo("Object of class MazeSolver deleted.")


def main():
    m = MazeSolver()
    m.solve_maze()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
