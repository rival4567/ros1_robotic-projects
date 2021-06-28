#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

import math

MX_RANGE = 10

FINAL_TARGET = Point()
FINAL_TARGET.x = 1.0
FINAL_TARGET.y = 2.0

YAW_PRECISSION = 5 * (math.pi / 180)  # 5 degrees
DIST_PRECISSION = 0.10


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('simbha_mover')

        self.regions = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0
        }

        self.velocity_msg = Twist()
        self.move_state = 0
        self.obstacle_avoidance_state = 0
        self.robot_state = 0
        self.reached_destination = False

        self.pose = [0.0, 0.0, 0.0]
        self.curr_position = Point()

        self.publisher_ = self.create_publisher(Twist, '/simbha_cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/simbha_bot/scan',
            self.laserscan_callback,
            10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # angle difference or yaw between any point and the bot.

    def error_angle(self, point):
        rx = self.pose[0]
        ry = self.pose[1]
        theta = self.pose[2]*(math.pi/180)
        theta_goal = math.atan2(point.y - ry, point.x - rx)
        error = self.normalize_angle(theta_goal - theta)
        return error

    # function that gives linear distance between any point and the bot.
    def calc_dist_points(self, point1, point2):
        dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
        return dist

    # function to limit the bot rotation between -pi to pi
    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_yaw(self, des_pos):
        err_yaw = self.error_angle(des_pos)

        if math.fabs(err_yaw) > YAW_PRECISSION:
            if err_yaw > 0:
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = 0.5
            else:
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = -0.5

        if math.fabs(err_yaw) < YAW_PRECISSION:
            self.move_state = 1

        self.publisher_.publish(self.velocity_msg)

    def go_straight_ahead(self, des_pos):
        err_yaw = self.error_angle(des_pos)
        curr_pos = Point()
        curr_pos.x = self.pose[0]
        curr_pos.y = self.pose[1]
        err_pos = self.calc_dist_points(des_pos, curr_pos)

        if err_pos > DIST_PRECISSION:

            self.velocity_msg.linear.x = 0.55
            self.velocity_msg.angular.z = 0.0 if err_yaw > YAW_PRECISSION else -0.0
            self.publisher_.publish(self.velocity_msg)
        else:
            self.done()

        # state change conditions
        if math.fabs(err_yaw) > YAW_PRECISSION:
            self.move_state = 0

    def done(self):
        self.velocity_msg.linear.x = 0.0
        self.velocity_msg.angular.z = 0.0
        self.publisher_.publish(self.velocity_msg)
        self.reached_destination = True

    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return [X, Y, Z]

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def odom_callback(self, data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.curr_position = data.pose.pose.position
        self.pose = [data.pose.pose.position.x,
                     data.pose.pose.position.y,
                     self.quaternion_to_euler(x, y, z, w)[2]]

    def reach_goal(self):

        if self.robot_state == 0:

            if self.move_state == 0:
                self.fix_yaw(FINAL_TARGET)
                self.get_logger().info('Fixing Heading to reach Target')
            elif self.move_state == 1:
                self.go_straight_ahead(FINAL_TARGET)
                self.get_logger().info('Moving towards the Target')

    def laserscan_callback(self, msg):
        self.regions = {
            'right': min(min(msg.ranges[0:143]), MX_RANGE),
            'fright': min(min(msg.ranges[144:287]), MX_RANGE),
            'front': min(min(msg.ranges[288:431]), MX_RANGE),
            'fleft': min(min(msg.ranges[432:575]), MX_RANGE),
            'left': min(min(msg.ranges[576:719]), MX_RANGE),
        }
        self.take_action(self.regions)

    def find_wall(self):

        self.velocity_msg.linear.x = 0.55
        self.velocity_msg.angular.z = -0.25
        self.publisher_.publish(self.velocity_msg)

    def turn_left(self):
        self.velocity_msg.linear.x = 0.0
        self.velocity_msg.angular.z = 0.55
        self.publisher_.publish(self.velocity_msg)

    def follow_the_wall(self):

        self.velocity_msg.linear.x = 0.35
        self.velocity_msg.angular.z = 0.0
        self.publisher_.publish(self.velocity_msg)
        # self.get_logger().info('%s'%str(self.velocity_msg))

    def take_action(self, regions):

        state_description = ''
        d = 1.50

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.obstacle_avoidance_state = 0

        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.obstacle_avoidance_state = 1

        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.obstacle_avoidance_state = 2

        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.obstacle_avoidance_state = 0

        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.obstacle_avoidance_state = 1

        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.obstacle_avoidance_state = 1

        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.obstacle_avoidance_state = 1

        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.obstacle_avoidance_state = 0

        else:
            state_description = 'unknown case'
            self.get_logger().info('Laser_scan_data: "%s"' % str(self.regions))

    def follow_wall(self):
        if self.obstacle_avoidance_state == 0:
            self.get_logger().info('Finding nearby wall')
            self.find_wall()
        elif self.obstacle_avoidance_state == 1:
            self.get_logger().info('Turning Left')
            self.turn_left()
        elif self.obstacle_avoidance_state == 2:
            self.get_logger().info('Following the wall')
            self.follow_the_wall()

    def timer_callback(self):
        self.follow_wall()

        # if self.reached_destination is True:
        #     self.get_logger().info('Target Reached')
        #     quit()
        # else:
        #     self.reach_goal()

        # self.get_logger().info('Odom: "%s"' % str(self.pose))
        # self.get_logger().info('Laser_scan_data: "%s"' % str(self.regions))


def main(args=None):
    rclpy.init(args=args)

    simbha_mover = ObstacleAvoider()

    rclpy.spin(simbha_mover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simbha_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
