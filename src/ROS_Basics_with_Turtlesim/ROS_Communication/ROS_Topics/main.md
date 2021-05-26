# ROS Topics

- ROS Topics allow **unidirectional** communication between ROS Nodes.

- When using ROS Topics a ROS Node can be a **publisher**, **subscriber** or both.

- A ROS Node acting as a publisher can publish data on a ROS Topic and a subscriber ROS Node can subscribe to a ROS Topic.

- Publisher and Subscriber Nodes will exchange ROS Messages over a ROS Topic.

- A ROS Message is a simple data structure, comprising typed fields (integer, floating point, boolean, etc.). So a ROS Message can hold data of various data-types.

- Consider this analogy,
    - Let's say you are subscribed to a newspaper called _The Noetic_ published by a publishing house called _OSRF_.

    - Every morning your paperboy _Jon Doe_ will deliver this newspaper to you.

    - You like _The Noetic_ because it has dedicated section on _sports_ and _robotics_ news.

    - In this analogy you can think,

        - `OSRF <--> ROS Publisher Node`
        OSRF which is publishing the newspaper as a Publisher Node.

        - `You <--> ROS Subscriber Node`

        You along with your neighbours who are subscribed to this newspaper as Subscriber Nodes. 

        - `Jon Doe <--> ROS Topic`
        Your paperboy who is taking the newspaper from the publisher and delivering it to its subscribers as a ROS Topic.

        - `The Noetic Newspaper <--> ROS Message`
        The physical newspaper is your ROS Message.
        
        - `Sports and Robotics Sections of The Noetic <--> Data Fields defined in ROS Message`
        The sections of the newspaper is the Data Fields defined in the ROS Message.

## Reading Assignment

1. [ROS Wiki - Topic](http://wiki.ros.org/Topics)
1. [ROS Wiki - Messages](http://wiki.ros.org/Messages)
---