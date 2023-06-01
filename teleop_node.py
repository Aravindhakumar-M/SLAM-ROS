#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
import curses
 
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)
direction = Int16MultiArray()

def teleop_node():
        char = screen.getch()
        if char == curses.KEY_RIGHT:
            direction=[0,0,1,0]
        elif char == curses.KEY_LEFT:
            direction=[0,0,0,1]       
        elif char == curses.KEY_UP:
            direction=[1,0,0,0]       
        elif char == curses.KEY_DOWN:
            direction=[0,1,0,0]
        pub=rospy.Publisher("/wheel", Int16MultiArray, queue_size=10)
        rospy.init_node('teleop_node', anonymous=True)
        curses.nocbreak(); screen.keypad(0); curses.echo()
        pub.publish(data=direction)
        print(direction)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():                   
            teleop_node()            
            # curses.endwin()
    except rospy.ROSInterruptException:
        pass