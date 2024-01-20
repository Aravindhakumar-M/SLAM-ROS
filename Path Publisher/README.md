# Path Planner ROS Node

The Path Planner ROS Node is a simple Python script that plans a path from a given start position to a goal position within a grid-based map. It subscribes to ROS topics for map data, initial pose, and goal point, and then plans a path based on this information.

## Prerequisites

To run the Path Planner ROS Node, you need to have the following installed:

- ROS Noetic
- Python 3
- NumPy
- ROS packages for standard message types (`std_msgs`, `nav_msgs`, `geometry_msgs`, `tf`, etc.)

## Usage

1. Make sure ROS is properly set up and running on your system.

2. Place this Python script in your ROS package directory.

3. Open a terminal and run the following command to start the ROS node:

   ```bash
   rosrun your_package_name path_pub.py
   ```

##   ROS Topics
The Path Planner ROS Node subscribes to the following ROS topics:

`/map (OccupancyGrid):` The map data used for path planning.  

`/initialpose (PoseWithCovarianceStamped):` The initial pose or start position.  

`/goal_point (PoseStamped):` The goal point or position to plan a path to.  

It also publishes the planned path as a ROS Path message on the topic `/path`.

## Configuration
The default start position is set to (0, 0) in the script. You can modify this to your desired start position in the start_callback function.

# Acknowledgments
Credit to the creators of ROS and its libraries.   
Extending gratitude to Anya Robotics Pvt Ltd for their valuable contributions and support.
