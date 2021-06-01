# Create a srv file

## Steps

1. Create a custom ROS Package or use any existing package like `pkg_ros_basics`.

1. Go into the package directory and create a folder called `srv`.

    ```bash
    roscd <package_name>
    mkdir srv
    ```

1. Instead of creating a new srv definition by hand, we will copy an existing one from another package. For that, `roscp` is a useful commandline tool for copying files from one package to another.
Usage:
    ```bash
    roscp [package_name] [file_to_copy_path] [copy_path]
    ```

    Now we can copy a service from the `rospy_tutorials` package:
    ```bash
    roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
    ```

    The srv file is,
    ```txt
    int64 a
    int64 b
    ---
    int64 sum
    ```

    Here `a` and `b` holds the request data which is sent by the Client to the Server and `sum` is the response which is sent by the Server to the Client.

    <center><a href="./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Services/AddTwoInts.srv" download><button>Download</button></a></center>

1. Open `package.xml`, and make sure these two lines are in it and uncommented:
    ```xml
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ```

1. Add the `message_generation` dependency to generate messages in `CMakeLists.txt`:
**Do not just add this line to your CMakeLists.txt, modify the existing line**
    ```txt
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    )
    ```

1. Remove `#` to uncomment the following lines:
    ```txt
    # add_service_files(
    #   FILES
    #   Service1.srv
    #   Service2.srv
    # )
    ```

    And replace the placeholder `<service_name>.srv` files for your service files:
    ```txt
    add_service_files(
    FILES
    AddTwoInts.srv
    )
    ```

1. Unless you have already done this in the previous steps, change in `CMakeLists.txt`. :
    ```txt
    # generate_messages(
    #   DEPENDENCIES
    # #  std_msgs  # Or other packages containing msgs
    # )
    ```
    Uncomment it and add any packages you depend on which contain `.msg` files that your messages use (in this case std_msgs), such that it looks like this:
    ```txt
    generate_messages(
    DEPENDENCIES
    std_msgs
    )
    ```

<center><a href="./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Services/CMakeLists.txt" download><button>Download CMakeLists.txt</button></a></center>


1. Go to `workspace` directory and run:
    ```bash
    catkin_make
    ```

## rossrv Command

The `rossrv` command-line tool displays information about ROS services. It has the exact same usage as `rosmsg` (see what it offers when it runs without sub-command below):

```bash
rossrv -h
rossrv show pkg_ros_basics/AddTwoInts
```

![rossrv-command.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Services/rossrv-command.png)
---