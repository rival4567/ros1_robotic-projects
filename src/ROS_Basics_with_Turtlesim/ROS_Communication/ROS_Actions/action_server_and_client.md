# Action Server & Client

  This tutorial covers using the simple_action_server library to create a Fibonacci action server in Python. This example action server generates a Fibonacci sequence, the goal is the order of the sequence, the feedback is the sequence as it is computed, and the result is the final sequence.

## 1. Creating the Action Messages

  Before writing an action it is important to define the goal, result, and feedback messages. The action messages are generated automatically from the .action file, for more information on action files see the [actionlib](https://wiki.ros.org/actionlib) documentation.

  Create actionlib_tutorials/action/Fibonacci.action in your favorite editor, and place the following inside it: 

  ```
  #goal definition
  int32 order
  ---
  #result definition
  int32[] sequence
  ---
  #feedback
  int32[] sequence
  ```

  To automatically generate the message files during the make process, a few things need to be added to CMakeLists.txt.

   - add the actionlib_msgs package to the find_package macro's argument like this (if you used catkin_create_package to generate CMakeLists.txt, this may already have been added): 

      ```
      find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
      ```

      - Note that CMake needs to find_package actionlib_msgs (message_generation does not need to be listed explicitly, it is referenced implicitly by actionlib_msgs).

    - use the add_action_files macro to declare the actions you want to be generated:

      ```
      add_action_files(
        DIRECTORY action
        FILES Fibonacci.action
      )
      ```

    - call the generate_messages macro, not forgetting the dependencies on actionlib_msgs and other message packages like std_msgs: 

      ```
      generate_messages(
        DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
      )
      ```

    - add actionlib_msgs to catkin_package macro like this: 

      ```
      catkin_package(
        CATKIN_DEPENDS actionlib_msgs
      )
      ```
      - catkin_package also specifies only CATKIN_DEPEND to actionlib_msgs. The transitive dependency on message_runtime is happening automatically. 

  Note: Sometimes you have to setup your package.xml, since we are generating messages you have to declare on the manifest file that at run time you have to generate messages. You could just insert the follow line. 

  ```
  <exec_depend>message_generation</exec_depend>
  
  ```
  also do not forget to add these in your package.xml.
  ```
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  ```

  now go to the root directory and run:

  ```bash
  catkin_make
  ```

  you can see the result by the following command:
  ```bash
  $ ls devel/share/actionlib_tutorials/msg/
  FibonacciActionFeedback.msg  FibonacciAction.msg        FibonacciFeedback.msg
  FibonacciResult.msg          FibonacciActionGoal.msg    FibonacciActionResult.msg  FibonacciGoal.msg
  $ ls devel/include/actionlib_tutorials/
  FibonacciActionFeedback.h  FibonacciAction.h        FibonacciFeedback.h  FibonacciResult.h
  FibonacciActionGoal.h      FibonacciActionResult.h  FibonacciGoal.h
  ```

## 2. Writing a Simple Server

  The code and examples used in this tutorial can be found in the [actionlib_tutorials](http://wiki.ros.org/actionlib_tutorials) package. You may want to read about the [actionlib](http://wiki.ros.org/actionlib) package before starting this tutorial. 

  The following code can be found in actionlib_tutorials/simple_action_servers/fibonacci_server.py, and implements a python action server for the fibonacci action.

  ```python
  #! /usr/bin/env python
   
  import rospy
   
  import actionlib
  
  import actionlib_tutorials.msg
  
  class FibonacciAction(object):
      # create messages that are used to publish feedback/result
      _feedback = actionlib_tutorials.msg.FibonacciFeedback()
      _result = actionlib_tutorials.msg.FibonacciResult()
  
      def __init__(self, name):
          self._action_name = name
          self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
          self._as.start()
        
      def execute_cb(self, goal):
          # helper variables
          r = rospy.Rate(1)
          success = True
          
          # append the seeds for the fibonacci sequence
          self._feedback.sequence = []
          self._feedback.sequence.append(0)
          self._feedback.sequence.append(1)
          
          # publish info to the console for the user
          rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
         
          # start executing the action
          for i in range(1, goal.order):
              # check that preempt has not been requested by the client
              if self._as.is_preempt_requested():
                  rospy.loginfo('%s: Preempted' % self._action_name)
                  self._as.set_preempted()
                  success = False
                  break
              self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
              # publish the feedback
              self._as.publish_feedback(self._feedback)
              # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
              r.sleep()
            
          if success:
              self._result.sequence = self._feedback.sequence
              rospy.loginfo('%s: Succeeded' % self._action_name)
              self._as.set_succeeded(self._result)
          
  if __name__ == '__main__':
      rospy.init_node('fibonacci')
      server = FibonacciAction(rospy.get_name())
      rospy.spin()

  ```

  make the script executable. For more information about the code refer [here](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29).

## 3. Compiling

  Only initially when you just created your tutorial package, you need to compile to generate shell config files.

  ```bash
  cd %TOPDIR_YOUR_CATKIN_WORKSPACE%
  catkin_make
  source devel/setup.bash
  ```

## 4. Writing a Simple Action Client

  The following code can be found in [actionlib_tutorials](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/scripts/fibonacci_client.py) repository, and implements a simple python action client for the fibonacci action.

  ```python
  #! /usr/bin/env python

  from __future__ import print_function

  import rospy
  # Brings in the SimpleActionClient
  import actionlib

  # Brings in the messages used by the fibonacci action, including the
  # goal message and the result message.
  import actionlib_tutorials.msg

  def fibonacci_client():
      # Creates the SimpleActionClient, passing the type of the action
      # (FibonacciAction) to the constructor.
      client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

      # Waits until the action server has started up and started
      # listening for goals.
      client.wait_for_server()

      # Creates a goal to send to the action server.
      goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

      # Sends the goal to the action server.
      client.send_goal(goal)

      # Waits for the server to finish performing the action.
      client.wait_for_result()

      # Prints out the result of executing the action
      return client.get_result()  # A FibonacciResult

  if __name__ == '__main__':
      try:
          # Initializes a rospy node so that the SimpleActionClient can
          # publish and subscribe over ROS.
          rospy.init_node('fibonacci_client_py')
          result = fibonacci_client()
          print("Result:", ', '.join([str(n) for n in result.sequence]))
      except rospy.ROSInterruptException:
          print("program interrupted before completion", file=sys.stderr)
  ```

  More details about the code [here](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29).

## Running the Client and Action Server

  Run:

  ```bash
  roscore
  ```

  Then on a new terminal, the following command will run the action server.

  ```bash
  rosrun actionlib_tutorials fibonacci_server.py
  ```

  On second terminal run:

  ```bash
  rosrun actionlib_tutorials fibonacci_client.py
  ```
### Reference

  - [ROS Tutorials Actionlib](http://wiki.ros.org/actionlib/Tutorials)

<hr>