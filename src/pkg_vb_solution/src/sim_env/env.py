'''
    env
    ====================
    This module includes all the utitlies such as logical camera, vacuum
    gripper, conveyor belt, 2D Camera to control the environment of
    Vargi Bots. This also includes method to change one kind of frame
    to another.

    *******************************************************************
    Author: Shivam  (kumarshivam4567@gmail.com)
    *******************************************************************
    Team ID: VB#0252
    -----------------
'''

import rospy
import geometry_msgs.msg

import tf2_ros

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from hrwros_gazebo.msg import LogicalCameraImage
# from pkg_task5.srv import Image2DCamera


class ControlSimEnv(object):
    '''This class is used to control the Vargi-bots simulations
        environment.``model_callback()`` detects packages under
        logical camera. Service calls to vacuum gripper, conveyor belt
        and 2D camera controls them.

    :param env_name: Environement name - `1` for shelf and `2` for bins
        defaults to `1`
    :type env_name: str, optional
    '''

    # pylint: disable = too-many-instance-attributes
    def __init__(self, env_name='1'):
        '''Constructor Method'''

        # Get all the parameters inside env
        param_config_env = rospy.get_param('env')

        # Store the parameters in variables
        logical_camera_topic_prefix = param_config_env['topic']['logical_camera_prefix']
        logical_camera_topic_suffix = param_config_env['topic']['logical_camera_suffix']
        vacuum_gripper_service_prefix = param_config_env['service']['vacuum_gripper_prefix']
        conveyor_belt_service_name = param_config_env['service']['conveyor_belt']
        # camera_1_service_name = param_config_env['service']['camera_1']
        connector = param_config_env['delimiter']

        self.ur5_arm = connector + env_name

        logical_camera_topic_name = logical_camera_topic_prefix + \
            env_name + logical_camera_topic_suffix + env_name
        vacuum_gripper_service_name = vacuum_gripper_service_prefix + "/" + self.ur5_arm

        # Subscribe to rostopic 'logical camera'
        self.logical_camera_sub = rospy.Subscriber(
            logical_camera_topic_name,
            LogicalCameraImage,
            self.model_callback,
            queue_size=1)
        # Get transform in buffer
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        # Using ROSService 'activate_vacuum_gripper' to control pick and
        # place of Vargi simulation robots.
        self.pick_box_srv = rospy.ServiceProxy(
            vacuum_gripper_service_name, vacuumGripper)
        rospy.loginfo(
            "Waiting for vacuum gripper service...")

        self.pick_box_srv.wait_for_service()

        rospy.loginfo("Connected to vacuum_gripper service!")

        # Using rosservice 'set_power' to control the conveyor belt power.
        # Power can be from 11 to 100.
        self.set_belt_power_srv = rospy.ServiceProxy(
            conveyor_belt_service_name, conveyorBeltPowerMsg)
        rospy.loginfo("Waiting for conveyor belt service...")
        self.set_belt_power_srv.wait_for_service()
        rospy.loginfo("Connected to conveyor belt service!")

        # Using rosservice 'capture_pkgs' to capture package colors
        # using 2D Camera
        # self.capture_pkgs_srv = rospy.ServiceProxy(
        #     camera_1_service_name, Image2DCamera)
        # rospy.loginfo("Waiting for 2D Camera service...")
        # self.capture_pkgs_srv.wait_for_service()
        # rospy.loginfo("Connected to 2D Camera service!")

        # Other class variables
        self.model_info = LogicalCameraImage().models
        self.model = ''
        self.model_pos = geometry_msgs.msg.Pose()
        self.logical_camera_pos = geometry_msgs.msg.Pose()
        self.pick_flag = False
        self.package_detected = False
        self.order_list = []

    def model_callback(self, model_msg):
        '''Callback to subscriber topic. This detects models
        under logical cameras from ROS topic

        :param model_msg: Custom message file imported from
            `hrwros_gazebo.msg.LogicalCameraImage` that is a list of
            models and their position and position of logical camera
        :type model_msg: hrwros_gazebo.msg.LogicalCameraImage
        '''
        self.model_info = model_msg.models
        self.logical_camera_pos = model_msg.pose
        # Save model type and model position is separate variables
        for mdl in range(len(model_msg.models)):
            self.model = model_msg.models[mdl].type
            self.model_pos = model_msg.models[mdl].pose
            # Check if package is in logical camera frustum
            self.package_detected = "package" in self.model

    def pick_package(self):
        '''Call vacuum gripper service in Activate form

        :return: `True`
        :rtype: bool
        '''
        return self.pick_box_srv.call(True).result

    def drop_package(self):
        '''Call vacuum gripper service and deactivate it.

        :return: `False`
        :rtype: bool
        '''
        return self.pick_box_srv.call(False).result

    def belt_speed(self, value):
        '''Controls the speed of conveyor belt

        :param value: Conveyor belt speed. Provide value between
            11 and 100 to service to run successfully.
        :type value: int
        :return: `True` if successful, otherwise `False`
        :rtype: bool
        '''
        return self.set_belt_power_srv.call(value).result

    # def use_camera_2d(self, value):
    #     '''Calls 2D Camera service and capture packages from shelf

    #     :param value: input `True` to use the 2D camera
    #     :type value: bool
    #     :return: Custom message of include `header`, `count`
    #         and `package` of list including `name` and `color`
    #         of package.
    #     :rtype: pkg_task5.srv.Image2DCamera
    #     '''
    #     return self.capture_pkgs_srv.call(value).Pkg

    def calc_tf(self, arg_frame_1, arg_frame_2):
        '''This method finds transform between two coordinate frames

        :param arg_frame_1: Name of frame 1
        :type arg_frame_1: str
        :param arg_frame_2: Name of frame 2
        :type arg_frame_2: str
        :return: message show position in x, y, z position
            and x, y, z, w orientation (quaternion)
        :rtype: geometry_msgs.msg.Pose
        '''
        try:
            trans = self._tf_buffer.lookup_transform(
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

        except(tf2_ros.LookupException,  # pylint: disable=no-member
               tf2_ros.ConnectivityException,  # pylint: disable=no-member
               tf2_ros.ExtrapolationException):  # pylint: disable=no-member
            rospy.logerr("TF error")
            return None

    def package_pose_world_using_logical_camera(self):
        '''Find model position under logical camera and converts
            to world frame

        :return: message show position in x, y, z position
            and x, y, z, w orientation (quaternion)
        :rtype: geometry_msgs.msg.Pose
        '''
        pos = geometry_msgs.msg.Pose()
        # Logical Camera 1 position in world frame [-0.800, 7.000, 2.000]
        # x=-z, y=y, z=x
        pos.position.x = self.logical_camera_pos.position.x \
            + self.model_pos.position.z
        pos.position.y = self.logical_camera_pos.position.y \
            + self.model_pos.position.y
        pos.position.z = self.logical_camera_pos.position.z \
            - self.model_pos.position.x
        pos.orientation.x = self.model_pos.orientation.x
        pos.orientation.y = self.model_pos.orientation.y
        pos.orientation.z = self.model_pos.orientation.z
        pos.orientation.w = self.model_pos.orientation.w
        return pos

    @staticmethod
    def ee_world_conv(pos):
        '''This method converts End-Effector frame to world frame

        :param pos: Position of End-effector
        :type pos: geometry_msgs.msg.Pose
        :return: Position in world frame
        :rtype: geometry_msgs.msg.Pose
        '''
        temp = pos.position.x
        pos.position.x = -pos.position.z
        pos.position.y = temp
        pos.position.z = -pos.position.y

        return pos

    @staticmethod
    def calc_distance(pos1, pos2):
        '''This calculates difference between two position coordinates

        :param pos1: Position of first frame
        :type pos1: geometry_msgs.msg.Pose
        :param pos2: Position of second frame
        :type pos2: geometry_msgs.msg.Pose
        :return: Difference between two coordinate frames
        :rtype: tuple[geometry_msgs.msg.Pose.position]
        '''
        return (pos1.position.x - pos2.position.x, pos1.position.y - pos2.position.y,
                pos1.position.z - pos2.position.z)

    # Destructor
    def __del__(self):
        '''Destructor Method
        '''
        # Unregister from rostopic 'logical_camera'
        self.logical_camera_sub.unregister()
        rospy.loginfo(
            '\033[94m' + "Object of class controlSimEnv Deleted." + '\033[0m')
