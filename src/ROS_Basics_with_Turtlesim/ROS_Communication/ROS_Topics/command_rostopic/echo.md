# echo

- `rostopic echo` shows the data published on a topic. 

- Usage:

```bash
rostopic echo [topic]
```

- In the earlier section, at the end, we used `-r` argument to keep it rotation at an angular velocity of 0.5 units. But what if the velocity is unknown and we need this information as feedback to control the motion of turtle???

- Our desire here is to get the pose information or simply one or all of the turtle's x,y, and z values w.r.t to the world.

- But let's see if there is any data being published by the turtlesim node in the first place. To do so, we'll use the following command...

```bash
rostopic list -p
```

![rostopic-list-p.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/rostopic-list-p.png)

- From the `-p` we know 2 topic is being published

    - `/turtle1/color_sensor`
    - `/turtle1/pose`

- Let's see more into the `/turtle1/pose` topic. 

![rostopic-info-turtle1-pose.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/rostopic-info-turtle1-pose.png)

- Luckily the pose information of turtle from the turtlesim is being published on the topic `/turtle1/pose`. 

- To display the pose data, enter the following command...

```bash
rostopic echo /turtle1/pose
```

![rostopic-echo.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/rostopic-echo.png)

---