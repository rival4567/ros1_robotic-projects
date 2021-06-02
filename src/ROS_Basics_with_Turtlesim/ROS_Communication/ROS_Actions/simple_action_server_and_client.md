# Simple Action Server & Client

  - This tutorial covers using the simple_action_server library to create a Fibonacci action server in Python. This example action server generates a Fibonacci sequence, the goal is the order of the sequence, the feedback is the sequence as it is computed, and the result is the final sequence.

## Writing a Simple Server
  
  - The following code can be found in actionlib_tutorials/simple_action_servers/fibonacci_server.py, and implements a python action server for the [fibonacci action](http://docs.ros.org/api/actionlib_tutorials/html/classfibonacci__server_1_1FibonacciAction.html).

  ```python
  #! /usr/bin/env python3
   
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

  You can see more about this code [here](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29).

## Compiling

  Only initially when you just created your tutorial package, you need to compile to generate shell config files.

  ```bash
  cd %TOPDIR_YOUR_CATKIN_WORKSPACE%
  catkin_make
  source devel/setup.bash
  ```

## Running the Action Server

  Run: 

  ```bash
  roscore
  ```

  Then on a new terminal, the following command will run the action server.

  ```bash
  rosrun actionlib_tutorials fibonacci_server.py
  ```

## Writing a Simple Action Client
  
  The following code can be found in [actionlib_tutorials](http://wiki.ros.org/actionlib_tutorials) repository, and implements a simple python action client for the [fibonacci action](http://docs.ros.org/api/actionlib_tutorials/html/classfibonacci__server_1_1FibonacciAction.html).

  ```python
  #! /usr/bin/env python3
   
   import rospy
   from __future__ import print_function
   
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
  You can read more about the code [here](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29).

## Running the client

  Before running the client, we assume roscore ans Action server are already running from previous page.

  Start the client. It will start up, send a goal to the server, wait for the goal to complete, and then exit.
  
  ```bash
  rosrun actionlib_tutorials fibonacci_client.py
  ```

### Reference
  - [ROS Wiki](http://wiki.ros.org/actionlib/Tutorials).
