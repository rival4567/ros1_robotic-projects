#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

from collections import defaultdict
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
        yaw = euler[2]-math.pi/2
        self.yaw_ = self.normalize_angle(yaw)

    def clbk_laser(self, msg):
        self.sensor_ = {
            # mapping laser data from 0 to 2
            'right':  msg.ranges[2]*10,
            'front':  msg.ranges[1]*10,
            'left':   msg.ranges[0]*10,
        }

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def calc_distance(self, x, y):
        return math.sqrt(x**2 + y**2)

    def done(self):
        self.move_micromouse(0, 0)
        rospy.loginfo("reached centre of maze!")

    def run(self):
        turn_ahead = False
        if self.sensor_['front'] > 1:
            self.move_micromouse(1.0, 0)
        if math.isinf(self.sensor_['left']) or math.isinf(self.sensor_['right']):
            turn_ahead = True
            cur_yaw = self.yaw_
        if turn_ahead:
            while not math.isinf(self.sensor_['left']) < 0.01:
                self.move_micromouse(0.5, 0)
            else:
                while math.fabs(cur_yaw - self.yaw_) < math.pi/2:
                    self.move_micromouse(0, 0.2)
                turn_ahead = False
                self.move_micromouse(0, 0)

    def avoid_obstacles(self):
        rospy.loginfo(self.sensor_)
        print("yaw", self.yaw_)
        angular_z = -0.2 if self.yaw_ < 0 else 0.2
        if self.sensor_['front'] < 0.9:
            self.move_micromouse(0, 0)
        while self.sensor_['left'] < 0.9:
            self.move_micromouse(0, angular_z)
        while self.sensor_['right'] < 0.9:
            self.move_micromouse(0, -angular_z)

    def move_micromouse(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.pub_vel.publish(twist_msg)

    def solve_maze(self):
        # Infinite loop
        while not rospy.is_shutdown():
            if self.calc_distance(self.position_.x, self.position_.y) < 0.02:
                self.done()
            else:
                self.run()
                self.avoid_obstacles()
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
