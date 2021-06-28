#!/usr/bin/env python3
# To ensure that the relative path works
import time
import floodfill_main
from taskvar_h import *
import math
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import rospy
import os
import sys

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)


TURNSID_TO_ANGLE = {
    'right': -90,
    'left': 90,
    'back': 180,
    'forward': 0,
    'back_right': -180
}  # back_right: turn 180 clockwise
DIR_TO_ANGLE = {
    'north': 0,
    'west': 90,
    'south': 180,
    'east': 270
}  # measured anticlockwise

WALL_THRES_20 = 90  # 80*sec(20)     #use to calculate wall follow error
WALL_THRES_10 = 120  # 80*sec(10)     #detect the walls in wall following
WALL_THRES_0 = 140
JUNC_DETECT_FRONT_THRES = 200
JUNC_DETECT_SIDE_THRES = 165  # for left and right
CELL_LENGTH = 185
HALF_CELL_LENGTH = 90
ORIGIN_TO_WHEEL_LENGTH = 20
LINEAR_VEL = 0.4


def clbk_odom(msg):
    global rob
    rob.is_odometry_updated = True

    rob.previous_x_dist = rob.x_dist
    rob.previous_y_dist = rob.y_dist
    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    rob.x_dist = round(position_.x * 1000, 2)
    rob.y_dist = round(position_.y * 1000, 2)

    rob.current_speed = math.sqrt((rob.x_dist - rob.previous_x_dist)**2 +
                                  (rob.y_dist - rob.previous_y_dist)**2)

    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    rob.yaw_ = round(euler[2], 3)


def clbk_laser(msg):
    global rob
    rob.is_laser_updated = True
    rob.region = {
        '10_degree_right': msg.ranges[0] * 1000,
        '20_degree_right': msg.ranges[1] * 1000,
        '40_degree_right': msg.ranges[2] * 1000
    }


""" ROS init commands """
pub = rospy.Publisher('/micromouse/cmd_vel', Twist, queue_size=1)
sub_odom = rospy.Subscriber('/micromouse/odom', Odometry, clbk_odom)
sub = rospy.Subscriber('/micromouse/laser/scan', LaserScan, clbk_laser)
rospy.init_node('cmd_robot', anonymous=True)
rate = rospy.Rate(50)  # 40hz


def currentAngle():  # current yaw value in degrees   0,90,180,270 -- range
    global rob
    if rob.yaw_ >= 0:
        return (rob.yaw_ / math.pi) * 180
    else:
        return 360 + ((rob.yaw_ / math.pi) * 180)


def currentAngleZero():  # this is used when desired angle is set to zero     0,-90,180,90 -- range
    global rob
    return (rob.yaw_ / math.pi) * 180


# used to calculate error in wall following. speciality: at the south this is varing 90 ,180 to 270 , not 170,180,-170
def currentAngleWallFlw(crnt_dir):
    if crnt_dir != DIR_TO_ANGLE['south']:
        return currentAngleZero()
    else:
        return currentAngle()


def curretnDir():  # current direction it looking at
    current_angle = currentAngle()
    if current_angle <= 45 or current_angle > 315:
        return DIR_TO_ANGLE['north']
    elif current_angle <= 315 and current_angle > 225:
        return DIR_TO_ANGLE['east']
    elif current_angle <= 225 and current_angle > 135:
        return DIR_TO_ANGLE['south']
    elif current_angle <= 135 and current_angle > 45:
        return DIR_TO_ANGLE['west']


def desiredAngle(angle):  # returns the destination angle
    desired_angle = TURNSID_TO_ANGLE[angle] + curretnDir()
    if desired_angle == -90:
        desired_angle = 270
    elif desired_angle == -180:
        desired_angle = 180
    elif desired_angle == 450:
        desired_angle = 90
    elif desired_angle == 360:
        desired_angle = 0
    return desired_angle


def getLinearSpeed(desired_speed=0.4):
    global rob
    linear_speed = desired_speed
    current_speed_to_velocity = rob.current_speed * 0.4 / 1.9
    speed_error = current_speed_to_velocity - linear_speed
    linear_speed -= speed_error * 1
    linear_speed = min(linear_speed, 0.4)
    return linear_speed


def publishVelocity(linear_speed, angular_speed):
    global pub
    msg1 = Twist()
    msg1.linear.x = linear_speed
    msg1.angular.z = angular_speed
    pub.publish(msg1)


# doesnt include 180 turn(back turn) important: 0.2 is used for take the position of the axel of the robot
def curveTurnNew(turn_direction):
    global rob
    crnt_dir = curretnDir()
    origin_x = 0
    origin_y = 0
    destination = 0  # this may be a x or y coordinate
    # this is used to choose one turn out of two types. maybe increasing(1) or decreasing(0)
    turn_flag = "none"
    sign = 0  # 1+1 or -1 denotes wheather right or left
    Kp = 0.15

    CURVE_TURN_INIT_SPEED = 0.12
    CURVE_TURN_ANGULAR_INIT_SPEED = 0.23
    MAX_ANGULAR_LIMIT = 0.8

    if crnt_dir == DIR_TO_ANGLE['north']:
        origin_y = rob.y_dist + ORIGIN_TO_WHEEL_LENGTH

        if turn_direction == 'right':
            origin_x = rob.x_dist - HALF_CELL_LENGTH
            turn_flag = 'decreasing'
            sign = -1
            destination = origin_x - ORIGIN_TO_WHEEL_LENGTH
        elif turn_direction == 'left':
            origin_x = rob.x_dist + HALF_CELL_LENGTH
            turn_flag = 'increasing'
            sign = 1
            destination = origin_x + ORIGIN_TO_WHEEL_LENGTH

    elif crnt_dir == DIR_TO_ANGLE['south']:
        origin_y = rob.y_dist - ORIGIN_TO_WHEEL_LENGTH

        if turn_direction == 'right':
            origin_x = rob.x_dist + HALF_CELL_LENGTH
            turn_flag = 'increasing'
            sign = -1
            destination = origin_x + ORIGIN_TO_WHEEL_LENGTH
        elif turn_direction == 'left':
            origin_x = rob.x_dist - HALF_CELL_LENGTH
            turn_flag = 'decreasing'
            sign = 1
            destination = origin_x - ORIGIN_TO_WHEEL_LENGTH

    if sign != 0:  # this turning is used for turns in x direction
        if turn_flag == 'increasing':
            while not rospy.is_shutdown():
                error = math.sqrt((rob.x_dist - origin_x)**2 +
                                  (rob.y_dist - origin_y)**2) - HALF_CELL_LENGTH
                angular_speed = (
                    CURVE_TURN_ANGULAR_INIT_SPEED + error * Kp) * sign

                if angular_speed > MAX_ANGULAR_LIMIT:
                    angular_speed = MAX_ANGULAR_LIMIT
                elif angular_speed < -MAX_ANGULAR_LIMIT:
                    angular_speed = -MAX_ANGULAR_LIMIT

                linear_speed = getLinearSpeed(CURVE_TURN_INIT_SPEED)
                publishVelocity(linear_speed, angular_speed)

                if destination < rob.x_dist:
                    publishVelocity(0, 0)
                    break

                rate.sleep()
        else:
            while not rospy.is_shutdown():
                error = math.sqrt((rob.x_dist - origin_x)**2 +
                                  (rob.y_dist - origin_y)**2) - HALF_CELL_LENGTH
                angular_speed = (
                    CURVE_TURN_ANGULAR_INIT_SPEED + error * Kp) * sign

                if angular_speed > MAX_ANGULAR_LIMIT:
                    angular_speed = MAX_ANGULAR_LIMIT
                elif angular_speed < -MAX_ANGULAR_LIMIT:
                    angular_speed = -MAX_ANGULAR_LIMIT

                linear_speed = getLinearSpeed(CURVE_TURN_INIT_SPEED)
                publishVelocity(linear_speed, angular_speed)

                if destination > rob.x_dist:
                    publishVelocity(0, 0)
                    break

                rate.sleep()
        return

    if crnt_dir == DIR_TO_ANGLE['west']:
        origin_x = rob.x_dist - ORIGIN_TO_WHEEL_LENGTH

        if turn_direction == 'right':
            origin_y = rob.y_dist - HALF_CELL_LENGTH
            turn_flag = 'decreasing'
            sign = -1
            destination = origin_y - ORIGIN_TO_WHEEL_LENGTH
        elif turn_direction == 'left':
            origin_y = rob.y_dist + HALF_CELL_LENGTH
            turn_flag = 'increasing'
            sign = 1
            destination = origin_y + ORIGIN_TO_WHEEL_LENGTH

    elif crnt_dir == DIR_TO_ANGLE['east']:
        origin_x = rob.x_dist + ORIGIN_TO_WHEEL_LENGTH

        if turn_direction == 'right':
            origin_y = rob.y_dist + HALF_CELL_LENGTH
            turn_flag = 'increasing'
            sign = -1
            destination = origin_y + ORIGIN_TO_WHEEL_LENGTH
        elif turn_direction == 'left':
            origin_y = rob.y_dist - HALF_CELL_LENGTH
            turn_flag = 'decreasing'
            sign = 1
            destination = origin_y - ORIGIN_TO_WHEEL_LENGTH

    if sign != 0:  # this turning is used for turns in y direction
        if turn_flag == 'increasing':
            while not rospy.is_shutdown():
                error = math.sqrt((rob.x_dist - origin_x)**2 +
                                  (rob.y_dist - origin_y)**2) - HALF_CELL_LENGTH
                angular_speed = (
                    CURVE_TURN_ANGULAR_INIT_SPEED + error * Kp) * sign

                if angular_speed > MAX_ANGULAR_LIMIT:
                    angular_speed = MAX_ANGULAR_LIMIT
                elif angular_speed < -MAX_ANGULAR_LIMIT:
                    angular_speed = -MAX_ANGULAR_LIMIT

                linear_speed = getLinearSpeed(CURVE_TURN_INIT_SPEED)
                publishVelocity(linear_speed, angular_speed)

                if destination < rob.y_dist:
                    publishVelocity(0, 0)
                    break

                rate.sleep()
        else:
            while not rospy.is_shutdown():
                error = math.sqrt((rob.x_dist - origin_x)**2 +
                                  (rob.y_dist - origin_y)**2) - HALF_CELL_LENGTH
                angular_speed = (
                    CURVE_TURN_ANGULAR_INIT_SPEED + error * Kp) * sign

                if angular_speed > MAX_ANGULAR_LIMIT:
                    angular_speed = MAX_ANGULAR_LIMIT
                elif angular_speed < -MAX_ANGULAR_LIMIT:
                    angular_speed = -MAX_ANGULAR_LIMIT

                linear_speed = getLinearSpeed(CURVE_TURN_INIT_SPEED)
                publishVelocity(linear_speed, angular_speed)

                if destination > rob.y_dist:
                    publishVelocity(0, 0)
                    break

                rate.sleep()
        return


def turn(turn_direction):
    global pub, rob
    desired_angle = desiredAngle(turn_direction)  # destination angle
    TURN_ANGULAR_SPEED = 1.2
    SPD_DECREMENT_ANGLE = 15
    # DECREMENT_CONST = (TURN_ANGULAR_SPEED - 0.2)/SPD_DECREMENT_ANGLE        #this equation makes sure that last angular speed doesnt go beyond 0.2

    if desired_angle != DIR_TO_ANGLE['north']:
        if (turn_direction == 'back') and (rob.region['left_most'] < rob.region['right_most']):
            turn_direction = 'back_right'  # turn back clockwise

        if TURNSID_TO_ANGLE[turn_direction] > 0:  # left or left 180
            while not rospy.is_shutdown():
                publishVelocity(0, TURN_ANGULAR_SPEED)
                # secondition is to prevent initial errors like zero being 359
                if (desired_angle - SPD_DECREMENT_ANGLE) < currentAngle() and (desired_angle + 90) > currentAngle():
                    break

                rate.sleep()

            while not rospy.is_shutdown():
                # decrementingAngularSpd = TURN_ANGULAR_SPEED - (SPD_DECREMENT_ANGLE - abs(abs(desired_angle) - abs(currentAngle()))) * DECREMENT_CONST
                decrementingAngularSpd = 0.3
                publishVelocity(0, decrementingAngularSpd)

                # secondition is to prevent initial errors like zero being 359
                if desired_angle < currentAngle() and (desired_angle + 90) > currentAngle():
                    publishVelocity(0, 0)
                    break

                rate.sleep()

        elif TURNSID_TO_ANGLE[turn_direction] < 0:  # right 90 or right 180
            while not rospy.is_shutdown():
                publishVelocity(0, -TURN_ANGULAR_SPEED)
                if (desired_angle + SPD_DECREMENT_ANGLE) > currentAngle() and (desired_angle - 90) < currentAngle():
                    break

                rate.sleep()

            while not rospy.is_shutdown():
                #decrementingAngularSpd = TURN_ANGULAR_SPEED - (SPD_DECREMENT_ANGLE - abs(abs(desired_angle) - abs(currentAngle()))) * DECREMENT_CONST
                decrementingAngularSpd = -0.3
                publishVelocity(0, decrementingAngularSpd)

                if desired_angle > currentAngle() and (desired_angle - 90) < currentAngle():
                    publishVelocity(0, 0)
                    break

                rate.sleep()

    else:
        if (turn_direction == 'back') and (rob.region['left_most'] < rob.region['right_most']):
            turn_direction = 'back_right'

        if TURNSID_TO_ANGLE[turn_direction] > 0:  # from 270 , 180 and 90 to 0
            while not rospy.is_shutdown():
                publishVelocity(0, TURN_ANGULAR_SPEED)
                if (desired_angle - SPD_DECREMENT_ANGLE) < currentAngleZero() and (desired_angle + 90) > currentAngleZero():
                    break

                rate.sleep()

            while not rospy.is_shutdown():
                #decrementingAngularSpd = TURN_ANGULAR_SPEED - (SPD_DECREMENT_ANGLE - abs(abs(desired_angle) - abs(currentAngleZero())))*DECREMENT_CONST
                decrementingAngularSpd = 0.3
                publishVelocity(0, decrementingAngularSpd)

                if desired_angle < currentAngleZero() and (desired_angle + 90) > currentAngleZero():
                    publishVelocity(0, 0)
                    break

                rate.sleep()

        elif TURNSID_TO_ANGLE[turn_direction] < 0:  # from 90 to zero
            while not rospy.is_shutdown():
                publishVelocity(0, -TURN_ANGULAR_SPEED)
                if (desired_angle + SPD_DECREMENT_ANGLE) > currentAngleZero() and (desired_angle - 90) < currentAngleZero():
                    break

                rate.sleep()

            while not rospy.is_shutdown():
                #decrementingAngularSpd = TURN_ANGULAR_SPEED - (SPD_DECREMENT_ANGLE - abs(abs(desired_angle) - abs(currentAngleZero()))) * DECREMENT_CONST
                decrementingAngularSpd = -0.3
                publishVelocity(0, decrementingAngularSpd)

                if desired_angle > currentAngleZero():
                    publishVelocity(0, 0)
                    break

                rate.sleep()


def wall_follow(
    Kp,
    Kd,
    crnt_dir,
    side,
    sign,
    desired_speed=LINEAR_VEL
):  # for right wall following sign must be +1 and for left wall it should be -1
    global rob  # becareful with linear speed

    rob.w_error = (WALL_THRES_20 - rob.region[side]) * \
        sign + (crnt_dir - currentAngleWallFlw(crnt_dir))
    if abs(rob.w_error) < 0.3:
        rob.w_error = 0

    rob.w_control_val = rob.w_error * Kp + \
        (rob.w_error - rob.w_last_error) * Kd
    rob.w_last_error = rob.w_error

    angular_speed = rob.w_control_val

    if angular_speed > 0.4:
        angular_speed = 0.4
    elif angular_speed < -0.4:
        angular_speed = -0.4

    publishVelocity(getLinearSpeed(desired_speed), angular_speed)


def noWallFollow(Kp, Kd, crnt_dir):  # if there is no wall go straight
    global rob
    rob.linear_speed = LINEAR_VEL

    rob.w_error = (crnt_dir - currentAngleWallFlw(crnt_dir))
    if abs(rob.w_error) < 0.3:
        rob.w_error = 0

    rob.w_control_val = rob.w_error * Kp + \
        (rob.w_error - rob.w_last_error) * Kd
    rob.w_last_error = rob.w_error

    angular_speed = rob.w_control_val

    if angular_speed > 0.4:
        angular_speed = 0.4
    elif angular_speed < -0.4:
        angular_speed = -0.4

    publishVelocity(getLinearSpeed(), angular_speed)


# when crnt_dir == west u should provide -90 instead of 270
def wallFollow(crnt_dir, desired_speed=0.4):
    global rob, WALL_THRES_10
    # checks wheather right wall is available
    if (rob.region['right_most'] <= WALL_THRES_0) and (rob.region['20_degree_right'] <= WALL_THRES_10):
        wall_follow(0.1, 0, crnt_dir, '20_degree_right', 1, desired_speed)
    elif (rob.region['left_most'] <= WALL_THRES_0) and (rob.region['20_degree_left'] <= WALL_THRES_10):
        wall_follow(0.1, 0, crnt_dir, '20_degree_left', -1, desired_speed)
    else:  # no wall is available to follow
        noWallFollow(0.1, 0, crnt_dir)


# provide the distance that you wan to move, in mm
def move_forward_specific_distance(distance):
    global rob
    init_given_distance = distance
    rob.linear_speed = LINEAR_VEL
    crnt_dir = curretnDir()

    if crnt_dir == 270:
        crnt_dir = -90  # this doesnt affect to the if conditions since it only look for 0 and 180. this is done to favour wall following code

    MAX_WALL_COUNT = 5
    left_wall_count = 0
    right_wall_count = 0
    LINEAR_VEL_AFTER_EDGE = 0.15
    DIST_AFTER_EDGE = 25

    if rob.is_fastrun_started:
        LINEAR_VEL_AFTER_EDGE = 0.15
        DIST_AFTER_EDGE = 20

    # need to use y cordinates
    if (crnt_dir == DIR_TO_ANGLE['north']) or (crnt_dir == DIR_TO_ANGLE['south']):
        initial_dist = rob.y_dist

        while not rospy.is_shutdown():
            wallFollow(crnt_dir, rob.linear_speed)
            """ code to detect wall edges """
            if rob.region['10_degree_left'] <= WALL_THRES_0:
                is_left_wall_temp = True
            else:
                is_left_wall_temp = False
            if rob.region['10_degree_right'] <= WALL_THRES_0:
                is_right_wall_temp = True
            else:
                is_right_wall_temp = False

            if is_left_wall_temp:
                left_wall_count += 1
            if is_right_wall_temp:
                right_wall_count += 1

            left_wall_count = min(left_wall_count, MAX_WALL_COUNT)
            right_wall_count = min(right_wall_count, MAX_WALL_COUNT)

            if (left_wall_count >= MAX_WALL_COUNT and not is_left_wall_temp):
                distance = abs(initial_dist - rob.y_dist) + DIST_AFTER_EDGE
                left_wall_count = 0
                rob.linear_speed = LINEAR_VEL_AFTER_EDGE

            elif (right_wall_count >= MAX_WALL_COUNT
                  and not is_right_wall_temp):
                distance = abs(initial_dist - rob.y_dist) + DIST_AFTER_EDGE
                right_wall_count = 0
                rob.linear_speed = LINEAR_VEL_AFTER_EDGE

            # sensor condition is added to stop robot in the middle of a cell, when turnOneCell function is called
            if (abs(initial_dist - rob.y_dist) >= distance) or (rob.region['front_sensor'] < 75):
                publishVelocity(0, 0)
                # if (rob.region['front_sensor'] < 75):
                #     print('stoped due to front sensor')
                break

            rate.sleep()

        if (distance != init_given_distance):
            #print('edge detected')
            rob.cell_error = 0
        else:
            rob.cell_error = (
                abs(initial_dist - rob.y_dist) - init_given_distance)

    else:
        initial_dist = rob.x_dist
        while not rospy.is_shutdown():
            wallFollow(crnt_dir, rob.linear_speed)
            """ code to detect wall edges """
            if rob.region['10_degree_left'] <= WALL_THRES_0:
                is_left_wall_temp = True
            else:
                is_left_wall_temp = False
            if rob.region['10_degree_right'] <= WALL_THRES_0:
                is_right_wall_temp = True
            else:
                is_right_wall_temp = False

            if is_left_wall_temp:
                left_wall_count += 1
            if is_right_wall_temp:
                right_wall_count += 1

            left_wall_count = min(left_wall_count, MAX_WALL_COUNT)
            right_wall_count = min(right_wall_count, MAX_WALL_COUNT)

            if (left_wall_count >= MAX_WALL_COUNT and not is_left_wall_temp):
                distance = abs(initial_dist - rob.x_dist) + DIST_AFTER_EDGE
                left_wall_count = 0
                rob.linear_speed = LINEAR_VEL_AFTER_EDGE

            elif (right_wall_count >= MAX_WALL_COUNT
                  and not is_right_wall_temp):
                distance = abs(initial_dist - rob.x_dist) + DIST_AFTER_EDGE
                right_wall_count = 0
                rob.linear_speed = LINEAR_VEL_AFTER_EDGE

            if abs(initial_dist - rob.x_dist) >= distance or (rob.region['front_sensor'] < 75):
                publishVelocity(0, 0)
                # if (rob.region['front_sensor'] < 75):
                #     print('stoped due to front sensor')  #important: remove this condition later
                break

            rate.sleep()

        if (distance != init_given_distance):
            # print 'edge detected'
            rob.cell_error = 0
        else:
            rob.cell_error = (
                abs(initial_dist - rob.x_dist) - init_given_distance)


def go_forward_and_turn(angle):  # angle == 'forward' state is neglected
    global rob
    #rob.is_fastrun_started = True
    if not rob.is_fastrun_started:
        if angle != 'back':
            move_forward_specific_distance(
                80 - rob.cell_error)  # 9cm   important
            rob.cell_error = 0  # cell error is put in here . cuz direction is changing
            turn(angle)
            move_forward_specific_distance(90)
        else:
            move_forward_specific_distance(87 - rob.cell_error)
            # turn back important:check wheather error correction is needed.
            turn(angle)
            move_forward_specific_distance(86)
    else:
        if angle != 'back':
            rob.cell_error = 0  # cell error is put in here . cuz direction is changing
            curveTurnNew(angle)
        else:
            move_forward_specific_distance(87 - rob.cell_error)
            # turn back important:check wheather error correction is needed.
            turn(angle)
            move_forward_specific_distance(86)


def juncDetect():
    global rob
    if rob.region['40_degree_right'] < JUNC_DETECT_SIDE_THRES:
        rob.right_wall.append(1)
    else:
        rob.right_wall.append(0)
    if rob.region['40_degree_left'] < JUNC_DETECT_SIDE_THRES:
        rob.left_wall.append(1)
    else:
        rob.left_wall.append(0)
    if rob.region['front_sensor'] < JUNC_DETECT_FRONT_THRES:
        rob.front_wall.append(1)
    else:
        rob.front_wall.append(0)


# core function. includes all the this that shold be done within a cell
def wallFollowOneCell(direction):
    global CELL_LENGTH, rob
    if rob.is_robot_moving == 0:
        move_forward_specific_distance(110)
        rob.is_robot_moving = 1
    elif direction == 'forward':
        move_forward_specific_distance(CELL_LENGTH - rob.cell_error)
    else:
        go_forward_and_turn(direction)

    juncDetect()


def searchRun():
    floodfill_main.run_main_init()
    floodfill_main.run_main_go_center()
    floodfill_main.run_main_go_to_start()
    floodfill_main.run_main_go_center()
    # floodfill_main.run_main_go_to_start()


def fastRun():  # include curve turn
    global rob
    rob.is_fastrun_started = True
    floodfill_main.run_main_go_center()


if __name__ == '__main__':

    while not (rob.is_odometry_updated and rob.is_laser_updated):
        pass

    floodfill_main.detectOrientation()
    searchRun()
    # fastRun()
    move_forward_specific_distance(90)
    print('TASK END')
