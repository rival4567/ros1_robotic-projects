# Example #4: Launch Shell Script and ROS Node

## Aim

- To write a launch file called `web_node.launch` to open `e-yantra.org` in firefox and run `node_hello_ros.py` of `pkg_ros_basics`.

- You need to write a shell script called `webpage_launch.sh` to open `e-yantra.org` in firefox and save it in `launch` folder of `pkg_ros_basics`.

## Code

`webpage_launch.sh`
```shell
#!/bin/bash

# Store URL in a variable
URL1="https://www.e-yantra.org/"

# Print some message
echo "** Opening $URL1 in Firefox **"

# Use firefox to open the URL in a new window
firefox -new-window $URL1 
```

**NOTE**: You need to make this shell script an executable using `chmod` before using it in a launch file.

`web_node.launch`
```xml
<launch>
    
    <node pkg="pkg_ros_basics" type="webpage_launch.sh" name="webpage-launch" output="screen">
        <param name="cmd" value="$(find pkg_ros_basics)/launch/webpage-launch.sh"/>
    </node>
        
    <node pkg="pkg_ros_basics" type="node_hello_ros.py" name="node_hello_ros" output="screen"/>

</launch>
```

## Run Command
```bash
roslaunch pkg_ros_basics web_node.launch
```

## Output
```bash
... logging to /home/ey-pc/.ros/log/e4944c60-025e-11eb-9079-40ec993efb48/roslaunch-ey-pc-23774.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ey-pc:37667/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.7
 * /webpage_launch/cmd: /home/ey-pc/eyrc...

NODES
  /
    node_hello_ros (pkg_ros_basics/node_hello_ros.py)
    webpage_launch (pkg_ros_basics/webpage_launch.sh)

ROS_MASTER_URI=http://localhost:11311

process[webpage_launch-1]: started with pid [23790]
** Opening https://www.e-yantra.org/ in Firefox **
process[node_hello_ros-2]: started with pid [23793]
[webpage_launch-1] process has finished cleanly
log file: /home/ey-pc/.ros/log/e4944c60-025e-11eb-9079-40ec993efb48/webpage_launch-1*.log
[INFO] [1601399716.259020]: Hello World!
```

- The code is self-explanatory.
- If you are not able to understand the code feel free to seek help from us.
---