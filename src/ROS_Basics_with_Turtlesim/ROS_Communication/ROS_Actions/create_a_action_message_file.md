# Create an action message file

  Before writing an action it is important to define the goal, result, and feedback messages. The action messages are generated automatically from the `.action` file, for more information on action files see the [actionlib](https://wiki.ros.org/actionlib) documentation. This file defines the type and format of the goal, result, and feedback topics for the action. Create actionlib_tutorials/action/Fibonacci.action in your favorite editor, and place the following inside it:

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

  Now by following, automatically generate msg files of your action files, and also see the result. 

  ```bash
  $ cd ../.. # Go back to the top level of your catkin workspace
  $ catkin_make
  $ ls devel/share/actionlib_tutorials/msg/
  FibonacciActionFeedback.msg  FibonacciAction.msg        FibonacciFeedback.msg
  FibonacciResult.msg          FibonacciActionGoal.msg    FibonacciActionResult.msg  FibonacciGoal.msg
  $ ls devel/include/actionlib_tutorials/
  FibonacciActionFeedback.h  FibonacciAction.h        FibonacciFeedback.h  FibonacciResult.h
  FibonacciActionGoal.h      FibonacciActionResult.h  FibonacciGoal.h
  ```

  To manually generate the message files from this file, use the script genaction.py from the actionlib_msgs package. 

### Reference
  - [actionlib_tutorial](https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer(ExecuteCallbackMethod)#actionlib_tutorials.2BAC8-Tutorials.2BAC8-SimpleActionServer.28ExecuteCallbackMethod.29.2BAC8-groovy.Creating_the_Action_Messages)
  
<hr>