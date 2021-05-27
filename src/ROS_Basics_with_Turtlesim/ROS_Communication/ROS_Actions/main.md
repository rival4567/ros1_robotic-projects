# ROS Actions

  - In any large ROS based system, here are cases when someone would like to send a request to a node to perform some task, and also receive a reply to the request. This can currently be achieved via `ROS services`. 
  - However, in some cases the sevices takes a long time to execute.
  - The user might want the ability to cancel the request during execution or get periodic feedback about how the request is progressing.
  - The `actionlib` package provides tools to create servers that execute long-running goals that can be preempted
  - It also provides a client interface in order to send requests to the server.
  - The action specification is defined using a `.action` file. The `.action` file has the goal definition, followed by the result definition, followed by the feedback definition, with each section separated by 3 hyphens (---). 

## Reference

  - More details about how actionlib operates "under the hood" are [here](http://wiki.ros.org/actionlib/DetailedDescription)