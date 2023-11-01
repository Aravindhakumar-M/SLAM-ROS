#!/usr/bin/env python3

# Import necessary libraries and modules
import rospy
import math
from sensor_msgs.msg import JointState
import tf
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Float64

# Define global variables for position and orientation
global x
x = 0.0
global y
y = 0.0
global x1
x1 = 1.0
global y1
y1 = 1.0
global thet
thet = 0.0

# Callback function for processing joint state information
def py_node(msg):
    
    global x
    global y
    global x1
    global y1
    global thet

    # Extract joint names, positions, and velocities from the message
    joint_names = msg.name
    joint_positions = msg.position
    joint_velocities = msg.velocity

    # Define wheel parameters
    wheel_radius = 0.4
    wheel_separation = 2.2
    front_wheel_separation = 2.2
    rear_wheel_separation = 2.2

    # Extract positions of front left and right wheels
    front_left_wheel_position = msg.position[0]
    front_right_wheel_position = msg.position[1]

    # Calculate changes in wheel positions
    r1 = front_left_wheel_position
    r2 = front_right_wheel_position
    L = 2.2

    # Calculate the change in orientation and distance traveled
    theta = (r1 - r2) / L
    s = (theta / 2) * (r1 + r2)
    
    # Calculate the new x and y positions based on the orientation change
    x1 = x + s * math.cos(thet + theta / 2)
    y1 = y + s * math.sin(thet + theta / 2)
    
    # Check if the position has changed
    if (x != x1 or y != y1):
        theta = (r1 - r2) / L
        s = (theta / 2) * (r1 + r2)
        x1 = x + s * math.cos(thet + theta / 2)
        y1 = y + s * math.sin(thet + theta / 2)
        thet = theta
        x = x1
        y = y1

        # Publish a transformation between odom and world_link
        translation = (x, y, 0.0)
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        time = rospy.Time.now()
        child = "world_link"
        parent = "odom"
        tf_broadcaster.sendTransform(translation, rotation, time, child, parent)
    else:
        pass

# Function to initialize the node and subscribe to joint state information
def cmdvel():
    rospy.init_node('tf_pub', anonymous=True)
    print("Started TF Publisher node")
    rospy.Subscriber('/joint_states', JointState, py_node)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize the TF broadcaster
        tf_broadcaster = tf.TransformBroadcaster()
        cmdvel()
    except rospy.ROSInterruptException:
        pass
