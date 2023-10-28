#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
import curses

# Initialize the curses library for capturing keyboard input
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)
direction = Int16MultiArray()

def teleop_node():
    # Capture a character from the keyboard
    char = screen.getch()
    if char == curses.KEY_RIGHT:
        direction = [0, 0, 1, 0]  # Move right
    elif char == curses.KEY_LEFT:
        direction = [0, 0, 0, 1]  # Move left
    elif char == curses.KEY_UP:
        direction = [1, 0, 0, 0]  # Move forward
    elif char == curses.KEY_DOWN:
        direction = [0, 1, 0, 0]  # Move backward

    # Initialize the ROS node and publish the direction as Int16MultiArray
    pub = rospy.Publisher("/wheel", Int16MultiArray, queue_size=10)
    rospy.init_node('teleop_node', anonymous=True)
    
    # End curses input handling to return to normal terminal behavior
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()

    # Publish the direction command
    pub.publish(data=direction)
    print(direction)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            teleop_node()
    except rospy.ROSInterruptException:
        pass
