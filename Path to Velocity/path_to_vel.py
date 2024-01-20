#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist

# Callback function for processing path messages
def py_node(msg):
    
    # Extract path points from the received message
    path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
    
    # Extract the first pose for calculations
    pose = msg.poses[0].pose
    
    # Define maximum linear and angular velocities
    max_linear_vel = 0.5
    max_angular_vel = 1.0

    # Calculate the distance and angle to the target point
    distance = ((pose.position.x**2) + (pose.position.y**2)) ** 0.5
    angle = math.atan2(pose.position.y, pose.position.x)
    
    # Calculate the angular velocity based on the angle
    angular_vel = angle * 0.5
    
    # Limit the linear velocity to the maximum allowed
    linear_vel = min(distance, max_linear_vel)
    
    # Limit the angular velocity to the maximum allowed and ensure it's within the range
    angular_vel = max(min(angular_vel, max_angular_vel), -max_angular_vel)
    
    x = linear_vel
    y = angular_vel

    # Calculate individual wheel velocities
    x1 = (2 * x + 2.2 * y) / 2
    x2 = (2 * x - 2.2 * y) / 2
    cmdvel(x1, x2)

# Function to publish wheel velocities
def cmdvel(x1, x2):

    pub1 = rospy.Publisher('/cont1/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/cont2/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/cont3/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/cont4/command', Float64, queue_size=10)
    rate = rospy.Rate(10)

    # Continuously publish wheel velocities
    while not rospy.is_shutdown():
        pub1.publish(x1)
        pub2.publish(x1)
        pub3.publish(x2)
        pub4.publish(x2)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('cont_node', anonymous=True)
        print("Started Velocity Publisher node")
        rospy.Subscriber('path', Path, py_node)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
