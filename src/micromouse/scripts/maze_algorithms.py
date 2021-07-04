#! /usr/bin/env python3

# maintain maze state in 2d numpy array. Subscribe to micromouse/odom and micromouse/laser/scan.
# Iterate through all 360 degrees and find (x,y) from
# distances in micromouse/laser/scan topic. Add some constant for each
# iteration that a particular grid has been exposed.


import message_filters
from nav_msgs.msg import Odometry
from micromouse.msg import dest

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import tf

import math
import numpy as np

import maze_solver


class Maze:
    def __init__(self):
        # For debugging
        self.start_time = None

        # Maze parameters
        self.d = 0.012  # Wall thickness in m
        self.s = 0.18-self.d  # Side length of each grid square sans the walls
        self.N = 16  # Number of grid squares in one row
        # Ensure that: N*(s+d)+d = total side length of maze

        # Tunable heuristics
        self.confidence_threshold = 10
        self.lower_thres = 0.2
        self.upper_thres = 0.8
        self.center_precision = 0.04  # (metres)

        # Maze state and maze confidence matrices
        self.w = 2*self.N-1
        self.h = 2*self.N-1
        self.maze_size = [self.w, self.h]
        self.maze_state = np.zeros(self.maze_size)
        self.maze_state[1::2, 1::2] = 1  # Both i and j odd, lattice point
        self.maze_confidence = np.zeros(self.maze_size)
        self.maze_confidence[1::2, 1::2] = - \
            1  # Both i and j odd, lattice point
        # Both i and j even, empty grid cell
        self.maze_confidence[::2, ::2] = -1

        self.goal = None
        self.prev_goal = None

        # Remember start position
        self.start_pos = None
        self.first_callback = True

        # Switch between modes
        self.run_number = Int32()
        self.run_number.data = 1
        self.run_number_pub = rospy.Publisher(
            '/run_number', Int32, queue_size=1)
        self.run_number_pub.publish(self.run_number)

    def remap(self, t):
        '''
        remaps from range [-N*(s+d)/2, N*(s+d)/2] to [0, N]
        t can be either {x_wall, y_wall}
        '''
        return t/(self.s+self.d) + self.N/2

    def inverseRemap(self, T):
        return (T-self.N/2)*(self.s+self.d)

    def updateMazeConfidence(self, x, y):
        fx, ix = math.modf(x)
        fy, iy = math.modf(y)

        if (fx < self.lower_thres or fx > self.upper_thres) and (fy < self.lower_thres or fy > self.upper_thres):
            return

        elif (fx < self.lower_thres and ix == 0) or \
             (fx > self.upper_thres and ix == self.N-1) or \
             (fy < self.lower_thres and iy == 0) or \
             (fy > self.upper_thres and iy == self.N-1):
            return

        elif fx < self.lower_thres:
            X = ix*2 - 1
            Y = iy*2
        elif fx > self.upper_thres:
            X = ix*2 + 1
            Y = iy*2
        elif fy < self.lower_thres:
            X = ix*2
            Y = iy*2 - 1
        elif fy > self.upper_thres:
            X = ix*2
            Y = iy*2 + 1
        else:
            return

        X = int(X)
        Y = int(Y)

        # print("%f\t%f" % (X,Y))
        try:
            if self.maze_confidence[X, Y] == -1:
                rospy.logfatal("Trying to update non-updatable grid")
                return

            if (self.maze_confidence[X, Y] < self.confidence_threshold):
                self.maze_confidence[X, Y] += 1
                if (self.maze_confidence[X, Y] == self.confidence_threshold):
                    self.maze_state[X, Y] = 1
        except IndexError as error:
            # rospy.loginfo("IndexError on updating (%f,%f)" % (x,y))
            pass

    def mazeCallback(self, odom, scan):
        if (odom is not None and scan is not None):
            # rospy.loginfo("Update requested")

            # Extract robot's (x,y,theta) from micromouse/odom
            x_bot = odom.pose.pose.position.x
            y_bot = odom.pose.pose.position.y
            quaternion = (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w)
            rpy = tf.transformations.euler_from_quaternion(quaternion)
            # By default bot is facing -Y (East direction)
            theta_bot = rpy[2] - math.pi/2

            if self.run_number != 1:
                # Obtain (x,y) of laser end-points from /scan + /odom
                laser_ranges = scan.ranges
                no_of_points = len(laser_ranges)
                max_range = scan.range_max

                # print("Bot pose: (%f,%f,%f)" % (x_bot, y_bot, theta_bot*180/math.pi))

                for i in range(no_of_points):
                    angle = scan.angle_min + i*scan.angle_increment
                    radial_distance = laser_ranges[i]
                    # Eliminate infinity ranges(idk if this is actually needed, for safety)
                    if (radial_distance <= max_range):
                        x_wall = x_bot + radial_distance * \
                            math.cos(theta_bot + angle)
                        y_wall = y_bot + radial_distance * \
                            math.sin(theta_bot + angle)
                        # print("Wall(laser): (%f,%f,%f)" % (x_wall, y_wall, (theta_bot + angle)*180/math.pi))
                        # rospy.signal_shutdown("Debug over")
                        # return

                        # print("Global: %f\t%f" % (x_wall, y_wall))
                        x_wall_remapped = self.remap(x_wall)
                        y_wall_remapped = self.remap(y_wall)
                        # print("[0,N]: %f\t%f" % (x_wall_remapped, y_wall_remapped))

                        self.updateMazeConfidence(
                            x_wall_remapped, y_wall_remapped)

            # Breadth First Search

            # Coordinates in 16*16 representation
            # print("Bot pose: (%f,%f)" % (x_bot, y_bot))
            x_bot_remapped = self.remap(x_bot)
            y_bot_remapped = self.remap(y_bot)

            # Coordinates in 31*31 maze_state representation
            X = int(x_bot_remapped)*2
            Y = int(y_bot_remapped)*2
            rospy.loginfo("Position: (%d,%d)" % (X, Y))

            if self.first_callback:
                self.start_pos = [X, Y]
                self.start_time = rospy.get_time()
                self.first_callback = False
                # rospy.loginfo(self.start_pos)

            # Finding next point in bfs path
            if self.run_number.data % 2:
                # Odd numbered run
                # rospy.loginfo("From corner to center...")
                if X == 14 and Y == 14:
                    rospy.loginfo("Reached center(exploratory)!")
                    rospy.loginfo(rospy.get_time()-self.start_time)
                    # rospy.signal_shutdown("Time period over")
                    self.start_time = rospy.get_time()
                    self.run_number.data += 1
                    self.run_number_pub.publish(self.run_number)
                    return
                else:
                    goal_ = maze_solver.BFS(
                        self.maze_state, X, Y, 14, 14, self.w, self.h)

            else:
                # rospy.loginfo("Going back to corner...")
                if X == self.start_pos[0] and Y == self.start_pos[1]:
                    rospy.loginfo("Reached start corner")
                    rospy.loginfo(rospy.get_time()-self.start_time)
                    self.start_time = rospy.get_time()
                    self.run_number.data += 1
                    self.run_number_pub.publish(self.run_number)
                    return
                else:
                    goal_ = maze_solver.BFS(self.maze_state, X, Y, self.start_pos[0],
                                            self.start_pos[1], self.w, self.h)

            # Oh no!
            if goal_ == -1:
                rospy.logfatal("Path not found!")
                rospy.signal_shutdown("Too fast")
                return

            self.goal = [self.inverseRemap(T/2+0.5) for T in goal_]

            # Start from center of some square before pursuing new goal
            if self.prev_goal is not None and self.prev_goal != self.goal:
                desired_start = np.array(
                    [self.inverseRemap(T/2+0.5) for T in [X, Y]])
                actual_start = np.array([x_bot, y_bot])
                if np.sum(np.abs(desired_start - actual_start)) > self.center_precision:
                    self.goal = desired_start
                    return

            self.prev_goal = self.goal[:]
            # print("Bot pose: (%d,%d)" % (X, Y))
            # print("Goal: (%d,%d)" % (goal_[0],goal_[1]))
            # print("Goal: (%f,%f)" % (self.goal[0],self.goal[1]))

            # if (rospy.get_time()-self.start_time) > 70:
            #   print(self.maze_confidence[12:19,12:19])
            #   np.savetxt('debug-confidence.txt', self.maze_confidence, delimiter='\t', fmt='% .0f')
            #   np.savetxt('debug-state.txt', self.maze_state, delimiter='\t', fmt='% .0f')
            # rospy.signal_shutdown("Time period over")
            # return


if __name__ == '__main__':
    try:
        rospy.init_node('maze_algorithms_node')  # , disable_signals=True)
        maze_obj = Maze()
        # maze_obj.start_time = rospy.get_time()
        while (rospy.get_time() == 0):  # Wait until nodes have loaded completely
            pass

        rospy.loginfo("Creating subscribers...")
        odom_sub = message_filters.Subscriber('/micromouse/odom', Odometry)
        scan_sub = message_filters.Subscriber(
            '/micromouse/laser/scan', LaserScan)
        rospy.loginfo("Done")

        rospy.loginfo(
            "Creating time synchronizer between odom and laser scan...")
        ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, scan_sub], queue_size=1, slop=0.1)
        ts.registerCallback(maze_obj.mazeCallback)
        rospy.loginfo("Done")

        rospy.loginfo("Creating publisher...")
        dest_pub = rospy.Publisher('/micromouse/dest', dest, queue_size=1)

        loop_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # rospy.loginfo("Running")
            if maze_obj.goal is not None:
                msg = dest()
                msg.dest_x_coordinate = maze_obj.goal[0]
                msg.dest_y_coordinate = maze_obj.goal[1]
                dest_pub.publish(msg)
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
