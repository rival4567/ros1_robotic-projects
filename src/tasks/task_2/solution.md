# Solution

  > **Note**: Remove this md page from mdbook and delete scripts from src/tasks/task0/ It shows one of the possible solution with autoeval script.

  `task_2.launch`

  ```xml
  <launch>
    
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node">
        <param name="/turtlesim_node/background_r" value="34" />
        <param name="/turtlesim_node/background_g" value="139" />
        <param name="/turtlesim_node/background_b" value="34" />
        <param name="/background_r" value="34" />
        <param name="/background_g" value="139" />
        <param name="/background_b" value="34" />
    </node>

    <node name="DrawShape" type="shape_server.py" pkg="pkg_task2"/>
    <node name="shape_client_py" type="shape_client.py" pkg="pkg_task2"/>

    <arg name="record" default="false"/>
    <arg name="duration" default="100"/>
    <arg name="rec_name" default="record.bag"/>

    <group if="$(arg record)">
        <node name="rosbag_record_turtle" pkg="rosbag" type="record"
            args="record -O $(find pkg_task2)/bagfiles/$(arg rec_name) --duration=$(arg duration) --chunksize=10 /turtle1/cmd_vel /turtle1/pose /DrawShape/status /DrawShape/goal /DrawShape/feedback /DrawShape/result" output="screen" />
    </group>


  </launch>
  
  ```
  <center><a href="tasks/task_2/task_2.launch" download><button>Download_launch_file</button></a></center>

  `shape_server.py`

  ```python
  	#! /usr/bin/env python3

	import rospy
	import actionlib
	import pkg_task2.msg
	import math
	import angles
	from geometry_msgs.msg import Twist
	from turtlesim.msg import Pose

	class ShapeAction:
		# create messages that are used to publish feedback/result
		_feedback = pkg_task2.msg.DrawShapeFeedback()
		_result = pkg_task2.msg.DrawShapeResult()

		def __init__(self, name):
			self._action_name = name
			rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_cb)
			self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
			self.vel_msg = Twist()
			self._as = actionlib.SimpleActionServer(self._action_name, pkg_task2.msg.DrawShapeAction, execute_cb=self.execute_cb, auto_start = False)
			self._as.start()
			rospy.loginfo('Server Started.')

		def execute_cb(self, goal):
			success = True
			if goal.shape == 'square':
				# helper variables
				r = rospy.Rate(1)
				edge_len = math.sqrt(2)*goal.radius
				self._result.side_length = edge_len
				self._result.interior_angle = math.pi/2
				l_scale = 6.0
				a_scale = 6.0
				min_error = 0.00001
				
				# starting execution
				rospy.loginfo('Executing, drawing a square with edge length %f', edge_len)
				for edge in range(4): # drawing individual edges
					# check that preempt has not been requested by the client
					if self._as.is_preempt_requested():
						rospy.loginfo('%s: Preempted' % self._action_name)
						self._as.set_preempted()
						success = False
						break

					start_x = self.turtle_x
					start_y = self.turtle_y
					start_theta = self.turtle_theta

					rospy.loginfo('Rotating...')
					theta_error = angles.normalize_angle_positive(math.pi - self._result.interior_angle - (self.turtle_theta - start_theta))
					while (theta_error > min_error):
						self.vel_msg.linear.x = 0
						self.vel_msg.angular.z = theta_error*a_scale
						self.vel_pub.publish(self.vel_msg)
						theta_error = angles.normalize_angle_positive(math.pi - self._result.interior_angle - (self.turtle_theta - start_theta))

					rospy.loginfo('Forward march!')
					dis_error = edge_len - abs(math.sqrt((start_x - self.turtle_x)*(start_x - self.turtle_x) + (start_y - self.turtle_y)*(start_y - self.turtle_y)))
					while (dis_error > min_error):
						self.vel_msg.linear.x = dis_error*l_scale
						self.vel_msg.angular.z = 0
						self.vel_pub.publish(self.vel_msg)
						dis_error = edge_len - abs(math.sqrt((start_x - self.turtle_x)*(start_x - self.turtle_x) + (start_y - self.turtle_y)*(start_y - self.turtle_y)))
					
					self._feedback.percent = 25.0+25.0*edge
					self._as.publish_feedback(self._feedback)
			elif goal.shape == 'circle':
				circumference = 2*math.pi*goal.radius
				rospy.loginfo('Executing, drawing a circle with circumference %f', circumference)
				self.vel_msg.linear.x = circumference
				self.vel_msg.angular.z = 2*math.pi
				self.vel_pub.publish(self.vel_msg)
				self._result.side_length = circumference
				self._result.interior_angle = 0

			if success:
				rospy.loginfo('%s: Succeeded' % self._action_name)
				self._as.set_succeeded(self._result)

		def turtle_pose_cb(self, pose):
			self.turtle_x = pose.x
			self.turtle_y = pose.y
			self.turtle_theta = pose.theta
			self.l_vel = pose.linear_velocity
			self.a_vel = pose.angular_velocity

	if __name__ == '__main__':
		rospy.init_node('DrawShape')
		server = ShapeAction(rospy.get_name())
		rospy.spin()
  ```
  <center><a href="tasks/task_2/shape_server.py" download><button>Download_server_script</button></a></center>

  `shape_client.py`

  ```python
  	#! /usr/bin/env python3

	import rospy
	import actionlib
	import pkg_task2.msg

	def shape_client():
		client = actionlib.SimpleActionClient('DrawShape', pkg_task2.msg.DrawShapeAction)
		rospy.loginfo('client ready.... waiting for server')
		# Waits until the action server has started up and started
		# listening for goals.
		client.wait_for_server()

		# Creates a goal to send to the server
		rospy.loginfo('Sending gaol1...')
		goal = pkg_task2.msg.DrawShapeGoal()
		goal.shape = 'square'
		goal.radius = 1
		# Sends goal to the action server
		client.send_goal(goal)


		r = rospy.Rate(0.25)
		r.sleep()
		rospy.loginfo('Sending gaol2...')
		goal.shape = 'circle'
		goal.radius = 2
		client.send_goal(goal)

		# wait for ther server to finish performing the action
		client.wait_for_result()

		# return result
		return client.get_result()

	if __name__ == '__main__':
		try:
			# Initializes a rospy node so that the SimpleActionClient can
			rospy.init_node('shape_client_py')
			result = shape_client()
			rospy.loginfo("Result: side length = %f, interior angle = %f" % (result.side_length, result.interior_angle))
		except:
			rospy.loginfo('program interrupted before completion')
  
  ```
  <center><a href="tasks/task_2/shape_client.py" download><button>Download_client_script</button></a></center>
<hr>