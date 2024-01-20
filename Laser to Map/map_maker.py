#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import euler_from_quaternion

class MapMaker:
    def __init__(self):
        rospy.init_node('map_pub', anonymous=True)
        
        self.scan_data = None
        self.odom_data = None
        self.map_resolution = 0.05
        self.map_width = 800
        self.map_height = 800
        self.previous_map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Create an OccupancyGrid message to represent the map
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_width
        self.map_msg.info.height = self.map_height
        self.map_msg.info.origin.position.x = -self.map_width * self.map_resolution / 2
        self.map_msg.info.origin.position.y = -self.map_height * self.map_resolution / 2
        self.map_msg.info.origin.orientation.w = 1.0

        # Initialize path and set up publishers and subscribers
        self.path = []
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher('/path', PoseStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.spin()

    def scan_callback(self, scan_msg):
        # Callback for handling laser scan data
        self.scan_data = scan_msg

        if self.odom_data is not None:
            x = self.odom_data.pose.pose.position.x
            y = self.odom_data.pose.pose.position.y
            orientation_q = self.odom_data.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            current_map_data = np.copy(self.previous_map_data)

            for i in range(len(self.scan_data.ranges):
                if self.scan_data.ranges[i] < self.scan_data.range_max:
                    angle = self.scan_data.angle_min + i * self.scan_data.angle_increment + yaw
                    x_range = int((x + self.scan_data.ranges[i] * math.cos(angle) - self.map_msg.info.origin.position.x) / self.map_resolution)
                    y_range = int((y + self.scan_data.ranges[i] * math.sin(angle) - self.map_msg.info.origin.position.y) / self.map_resolution)

                    if x_range >= 0 and x_range < self.map_width and y_range >= 0 and y_range < self.map_height:
                        current_map_data[y_range, x_range] = 100

            self.previous_map_data = current_map_data
            self.map_data = current_map_data

            # Publish map
            self.map_msg.header.stamp = rospy.Time.now()
            self.map_msg.data = np.ravel(self.map_data).tolist()
            self.map_pub.publish(self.map_msg)

    def odom_callback(self, odom_msg):
        # Callback for handling odometry data
        self.odom_data = odom_msg

        if self.scan_data is not None:
            x = self.odom_data.pose.pose.position.x
            y = self.odom_data.pose.pose.position.y
            orientation_q = self.odom_data.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # Add the current position to the path
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = odom_msg.pose.pose.position.x
            pose.pose.position.y = odom_msg.pose.pose.position.y
            pose.pose.position.z = odom_msg.pose.pose.position.z
            pose.pose.orientation = odom_msg.pose.pose.orientation
            self.path.append(pose)

            # Publish the path
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = 'map'
            path_msg.poses = self.path
            self.path_pub.publish(path_msg)

if __name__ == '__main__':
    MapMaker()
