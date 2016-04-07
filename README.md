# Table Clearing Planning Library #

This project is about computing the block and onTop symbolic predicates for planning actions. 

It also has an API to process edges in order to fill the occluded sides. useful for the AGILE package in order to detect more precisely the action to do. 

Now the implementation is at the point to implement the ROS package in order to get some feasible grasping pose with the following strategy:

* For each object:
* * get some grasping pose and consider only the ones more or less vertical
* * with a polygonal mesh (or a simple bounxng box of the gripper model) detect what are the objects that obstacle the object to be grasped given  a certain grasping pose
* sdasdad
* * * 