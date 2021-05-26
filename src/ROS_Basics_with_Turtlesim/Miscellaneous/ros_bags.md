# ROS Bags

  A bag is a file format in ROS for storing ROS [message](http://wiki.ros.org/Messages) data. Bags -- so named because of their .bag extension -- have an important role in ROS, and a variety of tools have been written to allow you to store, process, analyze, and visualize them.

  U can read more about them [here](http://wiki.ros.org/Bags).

## Recording and playing back data

  In this section we will learn how to record data from a running ROS system into a .bag file, and then to play back the data to produce similar behavior in a running system

  First, execute the following commands in separate terminals:

  Terminal 1:

  ```bash
  roscore
  ```

  Terminal 2:

  ```bash
  rosrun turtlesim turtlesim_node
  ```

  Terminal 3:

  ```bash
  rosrun turtlesim turtle_teleop_key
  ```

  This will start two nodes - the turtlesim visualizer and a node that allows for the keyboard control of turtlesim using the arrows keys on the keyboard. If you select the terminal window from which you launched turtle_keyboard, you should see something like the following:

  ```bash
  Reading from keyboard
  ---------------------------
  Use arrow keys to move the turtle.
  ```

  Pressing the arrow keys on the keyboard should cause the turtle to move around the screen. Note that to move the turtle you must have the terminal from which you launched turtlesim selected and not the turtlesim window. 

  We now will record the published data. Open a new terminal window. In this window run the following commands:

  ```bash
  mkdir ~/bagfiles
  cd ~/bagfiles
  rosbag record -a
  ```

  Here we are just making a temporary directory to record data and then running rosbag record with the option -a, indicating that all published topics should be accumulated in a bag file. 

  Move back to the terminal window with turtle_teleop and move the turtle around for 10 or so seconds.

  In the window running rosbag record exit with a Ctrl-C. Now examine the contents of the directory ~/bagfiles. You should see a file with a name that begins with the year, date, and time and the suffix .bag. This is the bag file that contains all topics published by any node in the time that rosbag record was running. 

  Now that we've recorded a bag file using rosbag record we can examine it and play it back using the commands rosbag info and rosbag play. First we are going to see what's recorded in the bag file. We can do the info command -- this command checks the contents of the bag file without playing it back. Execute the following command from the bagfiles directory: 

  ```bash
  rosbag info <your bagfile>
  ```

  You should see something like:

  ```bash
  path:        2014-12-10-20-08-34.bag
  version:     2.0
  duration:    1:38s (98s)
  start:       Dec 10 2014 20:08:35.83 (1418270915.83)
  end:         Dec 10 2014 20:10:14.38 (1418271014.38)
  size:        865.0 KB
  messages:    12471
  compression: none [1/1 chunks]
  types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
               rosgraph_msgs/Log   [acffd30cd6b6de30f120938c17c593fb]
               turtlesim/Color     [353891e354491c51aabe32df673fb446]
               turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
  topics:      /rosout                    4 msgs    : rosgraph_msgs/Log   (2 connections)
               /turtle1/cmd_vel         169 msgs    : geometry_msgs/Twist
               /turtle1/color_sensor   6149 msgs    : turtlesim/Color
               /turtle1/pose           6149 msgs    : turtlesim/Pose
  ```

  This tells us topic names and types as well as the number (count) of each message topic contained in the bag file. We can see that of the topics being advertised that we saw in the rostopic output, four of the five were actually published over our recording interval. As we ran rosbag record with the -a flag it recorded all messages published by all nodes. 

  Lets play the bag file to reproduce behavior in the running system. First kill the teleop program that may be still running from the previous section - a Ctrl-C in the terminal where you started turtle_teleop_key. Leave turtlesim running. In a terminal window run the following command in the directory where you took the original bag file:

  ```bash
  rosbag play <your bagfile>
  ```

  In this window you should immediately see something like:
  
  ```bash  
  [ INFO] [1418271315.162885976]: Opening 2014-12-10-20-08-34.bag

  Waiting 0.2 seconds after advertising topics... done.

  Hit space to toggle paused, or 's' to step.
  ```
  Eventually the topic /turtle1/cmd_vel will be published and the turtle should start moving in turtlesim in a pattern similar to the one you executed from the teleop program. The duration between running rosbag play and the turtle moving should be approximately equal to the time between the original rosbag record execution and issuing the commands from the keyboard in the beginning part of the tutorial. You can have rosbag play not start at the beginning of the bag file but instead start some duration past the beginning using the -s argument. A final option that may be of interest is the -r option, which allows you to change the rate of publishing by a specified factor. If you execute:

  ```bash
  rosbag play -r 2 <your bagfile>
  ```

  You should see the turtle execute a slightly different trajectory - this is the trajectory that would have resulted had you issued your keyboard commands twice as fast. 
### Reference
  - [rosbag tutorial](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)
  <hr>