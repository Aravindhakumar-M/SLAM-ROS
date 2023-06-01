#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
import tf
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Float64

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

def py_node(msg):
        
    global x
    global y
    global x1
    global y1
    global thet
    
    # print(x,y,x1,y1)
    joint_names = msg.name
    joint_positions = msg.position
    joint_velocities = msg.velocity
    
    wheel_radius = 0.4
    wheel_separation = 2.2
    front_wheel_separation = 2.2
    rear_wheel_separation = 2.2
    front_left_wheel_position = msg.position[0]
    front_right_wheel_position = msg.position[1]
    
    # Δs = (Δθ / 2) * (r1 + r2)
    # Δθ = (r1 - r2) / L
    # x1 = x0 + Δs * cos(θ0 + Δθ / 2)
    # y1 = y0 + Δs * sin(θ0 + Δθ / 2)

    r1 = front_left_wheel_position
    r2 = front_right_wheel_position
    L = 2.2
    
    theta = (r1 - r2) / L
    s = (theta / 2) * (r1 + r2)
    x1 = x + s*math.cos(thet + theta/2)
    y1 = y + s*math.sin(thet + theta/2)
    
    if (x!=x1 or y!=y1):
        theta = (r1 - r2) / L
        s = (theta / 2) * (r1 + r2)
        x1 = x + s*math.cos(thet + theta/2)
        y1 = y + s*math.sin(thet + theta/2)
        thet = theta
        x = x1
        y = y1
        
        translation = (x, y, 0.0)
        print(x, y)
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        time = rospy.Time.now()
        child = "world_link"
        parent = "odom"
        tf_broadcaster.sendTransform(translation, rotation, time, child, parent)
        
    else:
        pass
           
    # theta = (rear_left_wheel_position + rear_right_wheel_position - front_left_wheel_position - front_right_wheel_position) * 3.14 * wheel_radius / (front_wheel_separation + rear_wheel_separation)

def cmdvel():
    rospy.init_node('tf_pub', anonymous=True)
    print("Started TF Publisher node")
    rospy.Subscriber('/joint_states', JointState, py_node)
    rospy.spin()    

if __name__ == '__main__':
    try:
        tf_broadcaster = tf.TransformBroadcaster()
        cmdvel()
    except rospy.ROSInterruptException:
        pass
