#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist

def py_node(data):
    x = data.linear.x
    y = data.angular.z

    # Calculate control commands for four motors based on x and y values
    x1 = (2 * x + 2.2 * y) / 2
    x2 = (2 * x - 2.2 * y) / 2

    pub1 = rospy.Publisher('/cont1/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/cont2/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/cont3/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/cont4/command', Float64, queue_size=10)
    
    pub1.publish(x1)
    pub2.publish(x1)
    pub3.publish(x2)
    pub4.publish(x2)

def cmdvel():
    rospy.Subscriber('/cmd_vel', Twist, py_node)
    rospy.spin()    

if __name__ == '__main__':
    try:
        rospy.init_node('cont_pub', anonymous=True)
        cmdvel()
    except rospy.ROSInterruptException:
        pass
