#! /usr/bin/env python3

from sim_env.env import ControlSimEnv
from ur5_moveit.ur5 import Ur5Moveit
import rospy

from hrwros_gazebo.msg import LogicalCameraImage
from geometry_msgs.msg import Pose

import threading


package_detected = False
model = ''
model_pos = Pose()
ur5_joint_angles_dict = {}


def clbk_logical_camera_2(model_msg):
    '''Callback to subscriber topic. This detects models
        under logical cameras from ROS topic

        :param model_msg: Custom message file imported from
            `hrwros_gazebo.msg.LogicalCameraImage` that is a list of
            models and their position and position of logical camera
        :type model_msg: hrwros_gazebo.msg.LogicalCameraImage
        '''
    global package_detected, model, model_pos
    model_info = LogicalCameraImage().models
    logical_camera_pos = Pose()
    model_info = model_msg.models
    logical_camera_pos = model_msg.pose
    # Save model type and model position is separate variables
    for mdl in range(len(model_msg.models)):
        model = model_msg.models[mdl].type
        model_pos = model_msg.models[mdl].pose
        # Check if package is in logical camera frustum
        package_detected = "package" in model


def main():
    '''Main Function: Initialize the node and start sorting of packages.'''

    global package_detected, model, model_pos, ur5_joint_angles_dict

    rospy.init_node('node_sort_packages')

    ur5_1 = Ur5Moveit('ur5_1')
    env_1 = ControlSimEnv('1')
    ur5_2 = Ur5Moveit('ur5_2')
    env_2 = ControlSimEnv('2')

    ur5_1.group.set_planning_time(5)
    ur5_1.group.set_planner_id("RRTConnect")
    ur5_2.group.set_planning_time(5)
    ur5_2.group.set_planner_id("RRTStar")
    sub_lc = rospy.Subscriber(
        "/eyrc/vb/logical_camera_2", LogicalCameraImage, clbk_logical_camera_2)

    ur5_joint_angles_dict = {
        "ur5_1_pick_home": ur5_1.list_degrees_to_radians([180, -28, 35, -97, -90, 0]),
        "ur5_2_pick_home": ur5_1.list_degrees_to_radians([180, -55, 84, -119, -90, 0]),
        "red_bin": ur5_1.list_degrees_to_radians([80, -50, 70, -110, -90, 0]),
        "green_bin": ur5_1.list_degrees_to_radians([180, -150, -40, -80, 90, 0]),
        "blue_bin": ur5_1.list_degrees_to_radians([-95, -50, 70, -110, -90, 0])
    }

    ur5_1.set_joint_angles(ur5_joint_angles_dict["ur5_1_pick_home"])
    ur5_2.set_joint_angles(ur5_joint_angles_dict["ur5_2_pick_home"])
    env_1.belt_speed(100)

    while not rospy.is_shutdown():
        if not package_detected:
            env_1.belt_speed(30)
            continue
        picked = False
        rospy.loginfo(model)

        if "00" in model or "02" in model or "11" in model or "20" in model:
            while not picked:
                if 0.39 < model_pos.position.y < 0.41 and package_detected:
                    env_1.belt_speed(0)
                    # while not env_1.package_detected:
                    #     ur5_1.ee_cartesian_translation(
                    #         model_pos.position.z, 0, 0)
                    picked = env_1.pick_package()

            ur5_1.hard_set_joint_angles(ur5_joint_angles_dict["green_bin"])
            picked = env_1.drop_package()
            ur5_1.set_joint_angles(ur5_joint_angles_dict["ur5_1_pick_home"])
        elif "01" in model or "10" in model or "12" in model:
            while not picked:
                if -0.21 < model_pos.position.y < -0.19 and package_detected:
                    env_2.belt_speed(0)
                    # while not env_2.package_detected:
                    #     ur5_2.ee_cartesian_translation(
                    #         model_pos.position.z, 0, 0)
                    picked = env_2.pick_package()

            ur5_2.hard_set_joint_angles(ur5_joint_angles_dict["green_bin"])
            picked = env_2.drop_package()
            ur5_2.set_joint_angles(ur5_joint_angles_dict["ur5_2_pick_home"])
        else:
            rospy.loginfo("Waiting for package!")

    sub_lc.unregister()


if __name__ == '__main__':
    main()
