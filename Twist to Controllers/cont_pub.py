#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist

# Callback function that receives Twist messages and calculates control commands
def py_node(data):
    x = data.linear.x
    y = data.angular.z

    # Calculate control commands for four motors based on x and y values
    x1 = (2 * x + 2.2 * y) / 2
    x2 = (2 * x - 2.2 * y) / 2

    # Create publishers for each motor's control topic
    pub1 = rospy.Publisher('/cont1/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/cont2/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/cont3/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/cont4/command', Float64, queue_size=10)
    
    # Publish the control commands to each motor
    pub1.publish(x1)
    pub2.publish(x1)
    pub3.publish(x2)
    pub4.publish(x2)

# Initialize the ROS node and subscribe to the cmd_vel topic
def cmdvel():
    rospy.Subscriber('/cmd_vel', Twist, py_node)
    rospy.spin()    

if __name__ == '__main__':
    try:
        # Initialize the ROS node with an anonymous name
        rospy.init_node('cont_pub', anonymous=True)

        # Start listening for Twist messages on the cmd_vel topic
        cmdvel()
    except rospy.ROSInterruptException:
        pass
