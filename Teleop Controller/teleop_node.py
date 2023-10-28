#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import curses

# Initialize the curses library for keyboard input
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

# Create a Twist message to control linear and angular velocities
twist_msg = Twist()

def teleop_node():
    # Read keyboard input
    char = screen.getch()
    
    # Map keyboard input to linear and angular velocities
    if char == curses.KEY_RIGHT:
        twist_msg.angular.z = -1.0  # Rotate right
    elif char == curses.KEY_LEFT:
        twist_msg.angular.z = 1.0   # Rotate left
    elif char == curses.KEY_UP:
        twist_msg.linear.x = 1.0    # Move forward
    elif char == curses.KEY_DOWN:
        twist_msg.linear.x = -1.0   # Move backward
    else:
        # Stop the robot if no key is pressed
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

    # Create a publisher to send Twist messages to control the robot
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node('teleop_node', anonymous=True)
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()

    # Publish the Twist message to control the robot
    pub.publish(twist_msg)
    print(twist_msg)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # Continuously run the teleop_node function to control the robot
            teleop_node()
    except rospy.ROSInterruptException:
        pass
