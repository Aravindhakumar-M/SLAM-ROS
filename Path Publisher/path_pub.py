#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import *
import random
from std_msgs.msg import Float64MultiArray

class PathPlanner:
    def __init__(self):
        self.map = None
        self.map_res = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.start_pos = (0, 0)
        self.goal_pos = None
        self.map_max = None
        self.map_min = None
        self.map_shape = None
        self.grid_size = None

        # Subscribe to relevant topics
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_callback, queue_size=1)
        rospy.Subscriber('/goal_point', PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)

    def map_callback(self, msg):
        # Callback for the map topic
        time_stamp = msg.header.stamp.to_sec()
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_res = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_min = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_max = (msg.info.origin.position.x + msg.info.width * msg.info.resolution,
                        msg.info.origin.position.y + msg.info.height * msg.info.resolution)
        self.map_shape = (int(round((self.map_max[0] - self.map_min[0]) / self.map_res)),
                         int(round((self.map_max[1] - self.map_min[1]) / self.map_res)))

        print("Map resolution: ", self.map_res)
        print("Map origin: ", self.map_origin)
        print("Map width: ", self.map_width)
        print("Map height: ", self.map_height)
        print("Map min: ", self.map_min)
        print("Map max: ", self.map_max)
        print("Map shape: ", self.map_shape)

    def start_callback(self, msg):
        # Callback for the initial pose topic
        time_stamp = msg.header.stamp.to_sec()
        self.start_pos = (0, 0)  # Set the start position

    def goal_callback(self, msg):
        # Callback for the goal point topic
        time_stamp = msg.header.stamp.to_sec()
        self.goal_pos = (msg.pose.position.x, msg.pose.position.y)

        # Plan a path from start to goal
        path = self.plan_path(self.start_pos, self.goal_pos, time_stamp)
        print("Goal: ", self.goal_pos)

        # Create and publish a ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for pose in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.from_sec(pose[2])
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

    def plan_path(self, start_pos, goal_pos, time_stamp):
        # Function to plan a simple path from start to goal

        path = []
        curr_pos = Float64MultiArray()
        next_pos = Float64MultiArray()
        curr_pos = self.start_pos
        t = time_stamp

        for i in range(0, 5):
            next_pos = (curr_pos[0] + 1, curr_pos[1] + 1, t)
            path.append(next_pos)
            curr_pos = next_pos[:2]
            t += 1
            next_pos = (curr_pos[0] + 1, curr_pos[1] - 1, t)
            path.append(next_pos)
            curr_pos = next_pos[:2]
            t += 1
        return path

if __name__ == "__main__":
    rospy.init_node('path_pub', anonymous=True)
    print("Started Path Publisher node")
    planner = PathPlanner()
    rospy.spin()
