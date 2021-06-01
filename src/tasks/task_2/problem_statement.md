# Problem Statement

  - The objective of this task is to Create Ros Action server and client with the help of which you will draw shapes inside the turtlesim window. For this Taks you have to make a square shape with radius (distance of centeroid from a vertex) = 2 units.

  - You are supposed to do this by creating a server side script `shape_server.py` and a client side script `shape_client.py` and also a `.action` file called `DrawShape.action`.

## Procedure

  1. First, create a package named `pkg_task2`, within your catkin workspace. Make sure you update your `CMakeLists.txt` and `package.xml` just like in previous tutorials.

  2. Also create a `action` folder and create `DrawShape.action` file. It should have a string named 'shape' and integer 'radius' as goals, float variables 'side_length' and 'interior_angle' as results. You can leave the feedback portion as empty if you want.

  3. After completing the `.action` file make sure you compile and source the package once again.

  ```bash
  cd ~/workspace
  catkin catkin_make
  source devel/setup.bash

  ```

  4. Within this package, you should have a `scripts` folder inside which you'll create two python scripts, named `shape_server.py` and `shape_client.py`.

  5. After completing the python script. Make it executable, if it isn't already. To do that, enter the following code.

  ```bash
  chmod +x ~/catkin_ws/src/pkg_task0/scripts/node_turtle_revolve.py

  ```

  6. Before executing make sure that `roscore` is running along with `turtlesim_node`. You can either run them in separate terminals or simply create a `task2.launch` file inside the `~/catkin_ws/src/pkg_task0/launch/` folder. Launch file can run multiple nodes unlike a python/cpp script. Run the launch file, enter:

  ```bash
  roslaunch pkg_task0 task0.launch 

  ```
  7. The client side script will give shape 'square' and radius '2' as goals to the server. The server should draw a square with the required dimensions and stop at the initial position.

## Hints

  1. If you are not comfortable with creating scripts from scratch you can always take help from previous tutorials covered.

  2. Use linear velocity to draw straight lines and angular velocity to rotate at vertices.

  3. You should try to publish the velocities until the errors are very low. For Example:

  ```python
  # This is just and example

  while (error_distance > minimum_error):
  	move towards objective
  	error_distance = objective.distance - current_position

  while (error_theta > minimum_error):
  	rotate towards objective
  	error_theta = objective.theta - current_theta

  ```

<hr>