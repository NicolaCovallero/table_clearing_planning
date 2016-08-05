## Table Clearing Planning Library ##

This project is about computing the block and onTop symbolic predicates for planning actions. 

It also has an API to process edges in order to fill the occluded sides. useful for the AGILE package in order to detect more precisely the action to do.  

For each object now the algorithm also knows where it can push an object and has a very simple but good heuristic method for grasping (it is good with the object we are going to deal with), it only miss to compute the grasp predicates.

Now the implementation is at the point to implement the ROS package in order to get some feasible grasping pose with the following strategy:

* For each object:
 * with a polygonal mesh (or a simple bounxng box of the gripper model) detect what are the objects that obstacle the object to be grasped given  a certain grasping pose

Then we could implement the easy version of the problem (only push action with the goal to separate them all), and execute it on gazebo.