# list

- `rostopic list` returns a list of all topics currently subscribed to and published.

- Let's see this command's actual output. In a new terminal start the roscore:

```bash
roscore
```

> **Note**: Do not close this terminal, otherwise you won't be able to communicate with the rosmaster while executing some of its features.

- And in another terminal, run:

```bash
rostopic list
```

This should give something like the following image, which is the ROS's default topics. 

![rostopic-list-before.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/rostopic_list_before.png)

- For this tutorial, we will also use turtlesim. Please run in a new terminal: 
```bash
rosrun turtlesim turtlesim_node
```

You will see a small blue box with probably a different turtle within it. 

![turtlesim-node.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/turtlesim_node.png)

- Now, check again the `rostopic list` command in another terminal and observe the topics being subscribed or published.
![rostopic-list-after.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/command_rostopic/rostopic_list_after.png)

> **Note**: You can know more about these listed topics, by simply adding arguments as a suffix to `rostopic list`. And to know which arguments does what type `rostopic list --help`. Go ahead and experiment.
---