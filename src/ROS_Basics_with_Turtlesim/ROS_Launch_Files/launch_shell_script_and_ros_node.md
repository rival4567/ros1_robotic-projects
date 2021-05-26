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

`node_hello_ros.py`

```python
#!/usr/bin/env python3

import rospy

def main():
    # 1. Make the script a ROS Node.
    rospy.init_node('node_hello_ros', anonymous=True)

    # 2. Print Hello World!
    rospy.loginfo("Hello World!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

> **NOTE**: You need to make these shell script and python script executable using `chmod` before using it in a launch file.

`web_node.launch`
```xml
<launch>
    
    <node pkg="pkg_ros_basics" type="webpage_launch.sh" name="webpage_launch" output="screen">
        <param name="cmd" value="$(find pkg_ros_basics)/launch/webpage_launch.sh"/>
    </node>
        
    <node pkg="pkg_ros_basics" type="node_hello_ros.py" name="node_hello_ros" output="screen"/>

</launch>
```

## Run Command
```bash
roslaunch pkg_ros_basics web_node.launch
```

## Output

![web_node.png](./ROS_Basics_with_Turtlesim/ROS_Launch_Files/web_node.png)

<br />
- The code is self-explanatory.
- If you are not able to understand the code feel free to seek help from us.
---