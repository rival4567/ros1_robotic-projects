# Example #3: Load YAML

## Aim

- To write a launch file to load `config_my.yaml` present in `pkg_ros_basics` package.
- Also launch the `node_param_get_set.py` ROS node after loading the YAML file.

## Code

`load_yaml.launch`
```xml
<launch>
    
    <rosparam file ="$(find pkg_ros_basics)/config/config_my.yaml"/>
        
    <node pkg="pkg_ros_basics" type="node_param_get_set.py" name="node_param_get_set" output="screen"/>

</launch>
```

<center><a href="ROS_Basics_with_Turtlesim/ROS_Launch_Files/load_yaml.launch" download><button type="button">Download</button></a></center>

## Run Command

```bash
roslaunch pkg_ros_basics load_yaml.launch
```

## Output

![load_yaml.png](./ROS_Basics_with_Turtlesim/ROS_Launch_Files/load_yaml.png)

<br />
- The code is self-explanatory.
- If you are not able to understand the code feel free to seek help from us.
---