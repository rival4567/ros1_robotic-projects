#! /usr/bin/env python3

import rospy
from sys import argv
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg

from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest, vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest, conveyorBeltPowerMsgResponse

from hrwros_gazebo.msg import LogicalCameraImage


class CartesianPath:
    '''This class moves the ur5 arm using MoveIt!'''

    # Constructor
    def __init__(self):
        # Initialize node anonymously
        rospy.init_node('node_t3', anonymous=True)

        # Initialize `moveit_commander`_ node
        moveit_commander.roscpp_initialize(argv)

        # Kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self._scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object. This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        self._planning_group = "ur5_1_planning_group"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in RViz:
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        # Creating a simple action client to send goals.
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        # Other class variables
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z, synchronous="True"):
        '''This method use cartesian path to translate in x, y, z direction. It takes 4
        arguments: trans_x, trans_y, trans_z, synchronous. trans x, y, z takes translation
        in metre. synchronous is of bool type which gives movement of arm synchronously 
        (smoother) or async'''

        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        fraction = 0.0
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold

        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan, wait=synchronous)

    def set_joint_angles(self, arg_list_joint_angles):
        '''Setting joint angles for 'ur5_1_planning_group' all links.'''

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        # Move to desired joint angles
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        # Calling `stop()` ensures that there is no residual movement.
        self._group.stop()

        return flag_plan

    def go_to_pose(self, arg_pose):
        '''Go to desired pose for the end-effector'''

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # Move to target position
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found."
                + '\033[0m')

        # Calling 'stop()' ensures that there is no residual movement.
        self._group.stop()
        # Clearing targets after planning of poses.
        self._group.clear_pose_targets()
        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


class tfEcho:
    '''This class provides tf2 transform between different coordinate frames.'''

    # Constructor
    def __init__(self):
        # Get transform in buffer
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    def calc_tf(self, arg_frame_1="world", arg_frame_2="ur5_wrist_3_link"):
        '''This method finds transform between two coordinate frames and return
        transform. It takes two coordinate frames as input.'''

        try:
            trans = self._tfBuffer.lookup_transform(
                arg_frame_1, arg_frame_2, rospy.Time())

            # Calculate transform
            ref_pos = geometry_msgs.msg.Pose()
            ref_pos.position.x = trans.transform.translation.x
            ref_pos.position.y = trans.transform.translation.y
            ref_pos.position.z = trans.transform.translation.z

            ref_pos.orientation.x = trans.transform.rotation.x
            ref_pos.orientation.y = trans.transform.rotation.y
            ref_pos.orientation.z = trans.transform.rotation.z
            ref_pos.orientation.w = trans.transform.rotation.w

            return ref_pos

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
            return None

    def ee_world_conv(self, pos):
        '''This method converts End-Effector frame to world and return position
        in world frame.'''
        temp = pos.position.x
        pos.position.x = -pos.position.z
        pos.position.y = temp
        pos.position.z = -pos.position.y

        return pos

    def calc_distance(self, pos1, pos2):
        '''This calculates distance between two position coordinates.'''
        return (pos1.position.x - pos2.position.x, pos1.position.y - pos2.position.y,
                pos1.position.z - pos2.position.z)

    # Destructor
    def __del__(self):
        rospy.loginfo(
            '\033[94m' + "Object of class tfEcho Deleted." + '\033[0m')


class controlSimEnv:
    '''This class is used to control the VB simulations environment.'''

    # Constructor
    def __init__(self):

        # Subscribe to rostopic /eyrc/vb/logical_camera_2
        self.topic_name = "/eyrc/vb/logical_camera_2"
        self.sub = rospy.Subscriber(self.topic_name, LogicalCameraImage,
                                    self.model_callback, queue_size=1)

        # Other class variables
        self.model_info = LogicalCameraImage().models
        self.model = ''
        self.model_pos = geometry_msgs.msg.Pose()
        self.pick_flag = False
        self.package_detected = False

        # Using ROSService 'activate_vacuum_gripper' to control pick and place of Vargi
        # simulation robots.
        self.pick_box_srv = rospy.ServiceProxy(
            "/eyrc/vb/ur5_1/activate_vacuum_gripper", vacuumGripper)
        rospy.loginfo(
            "Waiting for /eyrc/vb/ur5_1/activate_vacuum_gripper service...")
        self.pick_box_srv.wait_for_service()
        rospy.loginfo("Connected to vacuum_gripper service!")

        # Using rosservice 'set_power' to control the conveyor belt power. Power can be
        # from 11 to 100.
        self.set_belt_power_srv = rospy.ServiceProxy(
            "/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)
        rospy.loginfo("Waiting for /eyrc/vb/conveyor/set_power service...")
        self.set_belt_power_srv.wait_for_service()
        rospy.loginfo("Connected to set_power service!")

    def model_callback(self, model_msg):
        # Callback to subscriber topic

        self.model_info = model_msg.models
        # Save model type and model position is separate variables
        for m in range(len(model_msg.models)):
            self.model = model_msg.models[m].type
            self.model_pos = model_msg.models[m].pose

        # Check if package is in logical camera frustum
        self.package_detected = self.model == "packagen1" or self.model == "packagen2"\
            or self.model == "packagen3"

    # Destructor
    def __del__(self):
        # Unregister from /eyrc/vb/logical_camera_2
        self.sub.unregister()
        rospy.loginfo(
            '\033[94m' + "Object of class controlSimEnv Deleted." + '\033[0m')


def main():
    # Define object for classes
    ur5 = CartesianPath()
    tf = tfEcho()
    env = controlSimEnv()

    # Other variables
    task_finished = False
    reference_frame = "ur5_wrist_3_link"
    ref_pos = None

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19

    # Defined three home positions for ur5 arm using joint angles.
    ur5_1_home_front_joint_angles = [-0.3623247940276908, -2.9108065805926753, -0.09345202312839884,
                                     -1.7081302018947557, 1.5707962921838723, -0.36232479405483087]

    ur5_1_home_mid_joint_angles = [0.13687798614297986, -2.442375782343568, -1.0176829030835242,
                                   -1.251753817369103, 1.5704502299497376, 0.1371094617247337]

    ur5_1_home_back_joint_angles = [0.6059869639432289, -2.937214832510734, -0.0387824203391709,
                                    -1.7363916566123745, 1.5707961622937958, 0.6059869639191504]

    # Bin positions using joint angles
    ur5_1_red_bin_joint_angles = [1.4515139748708146, -1.32523128036701, 2.0544499833951804,
                                  -2.3000151909844897, -1.5707963285225661, -1.6900786787216378]

    ur5_1_green_bin_joint_angles = [0.011579996212777388, -1.1104214907925725, 1.7368595610765318,
                                    -2.1972342164116148, -1.5707964842223632, -3.1300126574775646]

    ur5_1_blue_bin_joint_angles = [1.7909925949020966, -1.8114511190700302, -2.061194590020448,
                                   -0.8397431095558687, 1.570796105764062, 1.7909925949520566]

    while not rospy.is_shutdown():
        # Infinite loop unless Ctrl + C or some error.

        # Wait for packages to spawn.
        rospy.sleep(10)

        # If any package is not detected go to front home position.
        while not env.package_detected:
            ur5.set_joint_angles(ur5_1_home_front_joint_angles)
            env.set_belt_power_srv.call(100)
            continue

        while not task_finished:
            if not env.package_detected:
                continue
            else:
                # If a package is detected slow the belt speed.
                env.set_belt_power_srv(11)

            # target frame holds the frame of packages namely packagen1,
            # packagen2 and packagen3.
            target_frame = "logical_camera_2_" + env.model + "_frame"

            # This loop gets rid of the TF error message and finds position
            # between two coordinate frames.
            while not ref_pos:
                ref_pos = tf.calc_tf(reference_frame, target_frame)

            # Converts to world position.
            world_pos = tf.ee_world_conv(ref_pos)

            # Move arm towards the package in x-direction
            if abs(world_pos.position.x) > 0.02:
                ur5.ee_cartesian_translation(world_pos.position.x, 0, 0)

            # Move EE towards the package in y-direction and if it is detected,
            # pick up the package.
            while True:
                ref_pos = tf.calc_tf(reference_frame, target_frame)
                world_pos = tf.ee_world_conv(ref_pos)

                # if the EE is lagging behind package.
                if world_pos.position.y < -0.02:
                    env.set_belt_power_srv(11)
                    ur5.ee_cartesian_translation(
                        0, world_pos.position.y - 0.20, 0)
                    continue

                # if the EE is ahead of package.
                elif world_pos.position.y > box_length:
                    env.set_belt_power_srv(30)
                    ur5.ee_cartesian_translation(0, 0.1, 0)
                    continue

                # Pick up the package if it's in within reach.
                elif abs(world_pos.position.y) < 0.02:
                    rospy.loginfo("PACKAGE DETECTED")
                    while not env.pick_flag:
                        env.pick_flag = env.pick_box_srv.call(True).result
                    ref_pos = None
                    break

                # Wait for EE to reach middle of package.
                else:
                    continue

            # Go to respective bins based on package picked up.
            if env.model == "packagen1":
                env.set_belt_power_srv(20)
                ur5.set_joint_angles(ur5_1_red_bin_joint_angles)
                env.set_belt_power_srv(11)

            elif env.model == "packagen2":
                env.set_belt_power_srv(20)
                ur5.set_joint_angles(ur5_1_green_bin_joint_angles)
                env.set_belt_power_srv(0)

            elif env.model == "packagen3":
                env.set_belt_power_srv(0)
                ur5.set_joint_angles(ur5_1_blue_bin_joint_angles)
                # Once all three packages are picked up. Get out of loop.
                task_finished = True
                rospy.loginfo(
                    '\033[94m' + 'Task-3 successfully completed.' + '\033[0m')

            # Drops the package in bin.
            env.pick_flag = env.pick_box_srv.call(False).result

            rospy.loginfo("Y POSITION: " + str(env.model_pos.position.y))
            # Go to different home positions based on location of package.
            if env.model_pos.position.y < 0.0:
                rospy.loginfo(
                    '\033[94m' + 'Going to Home Back position.' + '\033[0m')
                ur5.set_joint_angles(ur5_1_home_back_joint_angles)

            elif env.model_pos.position.y < 0.65 or env.model_pos.position.x < -0.8:
                rospy.loginfo(
                    '\033[94m' + 'Going to Home Middle position.' + '\033[0m')
                ur5.set_joint_angles(ur5_1_home_mid_joint_angles)

            elif not env.package_detected:
                # Wait till another package is detected.
                rospy.loginfo(
                    '\033[94m' + 'Going to Home Front position.' + '\033[0m')
                ur5.set_joint_angles(ur5_1_home_front_joint_angles)
                env.set_belt_power_srv(100)

        break
    del ur5
    del tf
    del env


if __name__ == '__main__':
    main()
