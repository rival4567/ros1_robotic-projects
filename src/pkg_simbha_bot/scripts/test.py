#!/usr/env/python3

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('velocity_publisher')
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = Twist()
#         msg.linear.x = 1.0
#         msg.angular.z = 1.0
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: Linear_velocity: "%f" Angular_velocity: "%f" ' % (msg.linear.x,msg.angular.z))
#         # self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Simbha_bot(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'simbha_cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear_velocity: "%f" Angular_velocity: "%f" ' %
                               (msg.linear.x, msg.angular.z))
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    simbha_bot_mover = Simbha_bot()

    rclpy.spin(simbha_bot_mover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     main() = MinimalPublisher()

#     rclpy.spin(simbha_bot_mover)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    main()
