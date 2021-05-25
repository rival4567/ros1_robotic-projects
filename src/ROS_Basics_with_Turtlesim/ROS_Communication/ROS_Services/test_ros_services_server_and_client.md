# Test ROS Services - Server and Client

> For in depth code details in python language [Click here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

> For in depth code details in Cpp language [Click here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

- We will use an example from `rospy_tutorials`.

## Steps

1. First, run `roscore` command in a new terminal.
    ```bash
    roscore
    ```

1. Then run this command in a new terminal:
    ```bash
    rosrun rospy_tutorials add_two_ints_server 
    ```

1. Run this command in the new terminal for adding integer 12 and integer 20:
    ```bash
    rosrun rospy_tutorials add_two_ints_client 12 20
    ```

## Output
```bash
rosrun rospy_tutorials add_two_ints_server
```

```bash
Returning [12 + 20 = 32]
Returning [12 + 20 = 32]
```

<br />

```bash
rosrun rospy_tutorials add_two_ints_client 12 20
```

```bash
Requesting 12+20
12 + 20 = 32
```

---