# ROS TF
  - `tf` is a package that lets the user keep track of multiple coordinate frames over time.
  - `tf` maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.
  - A robotic system typically has many 3D [coordinate frames](http://wiki.ros.org/geometry/CoordinateFrameConventions) that change over time, such as a world frame, base frame, gripper frame, head frame, etc.
  - tf keeps track of all these frames over time, and allows you to ask questions like:
    - Where was the head frame relative to the world frame, 5 seconds ago?
    - What is the pose of the object in my gripper relative to my base?
    - What is the current pose of the base frame in the map frame?
   - tf can operate in a `distributed system`. This means all the information about the coordinate frames of a robot is available to all ROS components on any computer in the system. There is `no central server` of transform information.

## Reference
   - [ROS-wiki](http://wiki.ros.org/tf)