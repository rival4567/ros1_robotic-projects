#!/usr/bin/env python3

'''
Spawn shapes at with random position and orientation when the simbhabot reaches
some specific location. This will prevent hard coding.
'''


import rospy
import time
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
import random
import rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


print("******************************************")

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

path = rospack.get_path('pkg_simbha_bot')
print(path)


# Searching for models
sdf_model1 = open(path + '/models/bowling_ball/model.sdf', 'r').read()
sdf_model2 = open(path + '/models/cube/model.sdf', 'r').read()
sdf_model3 = open(path + '/models/cylinder/model.sdf', 'r').read()
sdf_model4 = open(path + '/models/my_vehicle/model.sdf', 'r').read()
sdf_model5 = open(path + '/models/unit_sphere/model.sdf', 'r').read()

bot_pos = Point()


def odom_callback(msg):
    global bot_pos
    bot_pos = msg.pose.pose.position

    # Spawning model at a location


def create_model_request(sdf_model, modelname, px, py, pz, rr, rp, ry):
    model = deepcopy(sdf_model)
    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = model
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


# Shuffling and generating pose and orientation
def random_model_pose(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):
    global bot_pos
    x_position = [p1_x, p2_x, p3_x]
    y_position = [p1_y, p2_y, p3_y]
    random.shuffle(x_position)
    random.shuffle(y_position)
    i = round(random.uniform(0, 3.14), 2)

    pose1 = [bot_pos.x + x_position[0],
             bot_pos.y + y_position[0], 0.0, 0.0, 0.0, i]
    pose2 = [bot_pos.x + x_position[1],
             bot_pos.y + y_position[1], 0.0, 0.0, 0.0, i]
    pose3 = [bot_pos.x + x_position[2],
             bot_pos.y + y_position[2], 0.0, 0.0, 0.0, i]

    return pose1, pose2, pose3


if __name__ == '__main__':

    rospy.init_node('spawn_models')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    rospy.Subscriber(
        '/odom', Odometry, odom_callback)

    # shuffling pose
    # x1 y1 x2 y2 x3 y3  change this value for spawning position

    spawned_count = 0
    sphere_spawned = False
    cube_spawned = False
    cylinder_spawned = False
    while spawned_count < 3:
        pose1, pose2, pose3 = random_model_pose(
            1.108564, 1.239278, 0.71279, 1.3995, 1.34, 1.24)
        if (3.0 < bot_pos.x < 6.0) and (1.0 < bot_pos.y < 4.0) and not sphere_spawned:
            rospy.loginfo("Spawning Sphere")
            req1 = create_model_request(
                sdf_model5, "sphere1", pose2[0], pose2[1], pose2[2], pose2[3], pose2[4], pose2[5])  # theta
            spawn_srv.call(req1)
            spawned_count += 1
            sphere_spawned = True
        if (1.0 < bot_pos.x < 2.2) and (2.5 < bot_pos.y < 4.2) and not cube_spawned:
            rospy.loginfo("Spawning cube")
            req2 = create_model_request(
                sdf_model2, "cube", pose1[0], pose1[1], pose1[2], pose1[3], pose1[4], pose1[5])
            spawn_srv.call(req2)
            spawned_count += 1
            cube_spawned = True
        if (1.0 < bot_pos.x < 2.2) and (-4.0 < bot_pos.y < -2.5) and not cylinder_spawned:
            rospy.loginfo("Spawning cylinder")
            req3 = create_model_request(
                sdf_model3, "cylinder", pose3[0], pose3[1], pose3[2], pose3[3], pose3[4], pose3[5])
            spawn_srv.call(req3)
            spawned_count += 1
            cylinder_spawned = True
    rospy.loginfo("Spawned all models. Shutting down the node!")
    rospy.shutdown()
