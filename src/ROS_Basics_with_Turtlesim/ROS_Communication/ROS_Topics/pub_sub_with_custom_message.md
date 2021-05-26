# Example #1: Pub-Sub with Custom Message

## Aim

To write a `listener` and `talker` node which should communicate with each other over a ROS Topic called `my_topic` using a custom ROS Message called `myMessage` with the following data fields of the following data types.

1. int32 id
1. string name
1. float32 temperature
1. float32 humidity

## Steps

### Create Custom ROS Message

- Messages are just simple text files with a field type and field name per line. 

- They are stored in the `msg` directory of your package. 


1. Create a file and name it `myMessage.msg` and store it in a `msg` folder of `pkg_ros_basics`. If the folder does not exist create it.

1. Now fill the `myMessage.msg` file with the following content.

    ```txt
    int32 id
    string name
    float32 temperature
    float32 humidity
    ```
    This is the format of a typical `msg` file.

1. Now open your `package.xml` file of `pkg_ros_basics` package and add in the dependencies for your `geometry_msgs`, `message_generation` and `message_runtime` as seen below.

    ```xml
    <?xml version="1.0"?>
    <package format="2">
        <name>pkg_ros_basics</name>
        <version>0.0.0</version>
        <description>The pkg_ros_basics package</description>

        <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>

        <license>TODO</license>

        <buildtool_depend>catkin</buildtool_depend>
        <build_depend>roscpp</build_depend>
        <build_depend>rospy</build_depend>
        <build_depend>std_msgs</build_depend>
        <build_export_depend>roscpp</build_export_depend>
        <build_export_depend>rospy</build_export_depend>
        <build_export_depend>std_msgs</build_export_depend>
        <exec_depend>roscpp</exec_depend>
        <exec_depend>rospy</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        <build_depend>message_generation</build_depend>
        <exec_depend>message_runtime</exec_depend>
        <build_depend>geometry_msgs</build_depend>
        <exec_depend>geometry_msgs</exec_depend>

        <export>
        </export>
    </package>
    ```

1. Now open your `CMakeList.txt` file of `pkg_ros_basics` package and navigate to the following block of code in your file.

    ```xml
    # add_message_files(
    #   FILES
    #   Message1.msg
    #   Message2.msg
    # )
    ```

    Uncomment the Messages and add include the name of your Message files. You can include multiple Message files if required as well.

    Now your `CMakeList.txt` should look like this,

    ```txt
    cmake_minimum_required(VERSION 3.0.2)
    project(pkg_ros_basics)


    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
    )

    add_message_files(
        FILES
        myMessage.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
    )

    catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES pkg_ros_basics
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
    #  DEPENDS system_lib
    )

    ###########
    ## Build ##
    ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
    # include
        ${catkin_INCLUDE_DIRS}
    )
    ```

1. After this build your package.

    ```bash
    cd ~/workspace
    catkin_make
    ```

    Once the package is build successfully you can see `myMessage.h` file located at `~/workspace/devel/include/pkg_ros_basics/myMessage.h`. This will be used by ROS Nodes to communicate over a ROS Topic using `myMessage` ROS Message. 


## Code - ROS Nodes

### Listener Node

`node_myMsg_listener.py`

```python
#!/usr/bin/env python3

import rospy
from pkg_ros_basics.msg import myMessage


def func_callback_topic_my_topic(myMsg):

    rospy.loginfo("Data Received: (%d, %s, %.2f, %.2f)", myMsg.id,
                  myMsg.name, myMsg.temperature, myMsg.humidity)


def main():

    # 1. Initialize the Subscriber Node.
    rospy.init_node('node_myMsg_listener', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("my_topic", myMessage, func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

### Talker Node

`node_myMsg_talker.py`

```python
#!/usr/bin/env python3

import rospy
from pkg_ros_basics.msg import myMessage

import random


def main():
    
    # 1. Create a handle to publish messages to a topic.
    var_handle_pub = rospy.Publisher('my_topic', myMessage, queue_size=10)
    
    # 2. Initializes the ROS node for the process.
    rospy.init_node('node_myMsg_talker', anonymous=True)

    # 3. Set the Loop Rate 
    var_loop_rate = rospy.Rate(1) # 1 Hz : Loop will its best to run 1 time in 1 second
    
    # 4. Write the infinite Loop
    while not rospy.is_shutdown():
        obj_msg = myMessage()

        obj_msg.id = 1
        obj_msg.name = "my_message"
        obj_msg.temperature = 10 + random.random()
        obj_msg.humidity = 20 + random.random()

        rospy.loginfo("Publishing: ")
        rospy.loginfo(obj_msg)

        var_handle_pub.publish(obj_msg)

        var_loop_rate.sleep()



# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## Output

For analyzing the output for these custom messages, you can follow the following steps  

1. `roscore` - As seen in previous tutorials, you must have a roscore running for the nodes to communicate. To view the messages between the talker and listener nodes, run an instance of roscore in a separate terminal window

1. listener node - For making the script of your node executable run `chmod +x` within the appropriate directory in a separate terminal window other than where your roscore is running. To run the listener node, run the following commands within your appropriate directory `rosrun <package_name> <listener_node.py>`

1. talker node - You can follow the same steps mentioned above for running your talker node using the commands `rosrun <package_name> <talker_node.py>`

If you follow the steps given above, you should see the following output 

```bash
rosrun pkg_ros_basics node_myMsg_talker.py 
```

![node-myMsg-talker.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/node-myMsg-talker.png)

<br />  
```bash
rosrun pkg_ros_basics node_myMsg_listener.py
```

![node-myMsg-listener.png](./ROS_Basics_with_Turtlesim/ROS_Communication/ROS_Topics/node-myMsg-listener.png)

- If you face any problems while following this tutorial you can feel free to reach out to us. 
---