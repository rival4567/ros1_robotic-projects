#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


def clbk_cam(msg):
    points = PointCloud2()
    points.height = msg.height
    points.width = msg.width
    points.point_step = msg.point_step
    points.row_step = msg.row_step
    points.data = msg.data
    points.fields = msg.fields
    # print(points.point_step, points.row_step)
    print(len(points.data))


def main():
    rospy.init_node('reading_depth_cam_points')

    sub = rospy.Subscriber('/camera/depth/points', PointCloud2, clbk_cam)

    rospy.spin()


if __name__ == '__main__':
    main()
