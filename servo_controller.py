#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
   
def cont():
    
    
    pos=list(map(Float64, input("Enter 5 position values as ssv : ").strip().split()))
    print(pos)
   
    pos_pub1 = rospy.Publisher('/cont1/command', Float64, queue_size=10)
    pos_pub2 = rospy.Publisher('/cont2/command', Float64, queue_size=10)
    pos_pub3 = rospy.Publisher('/cont3/command', Float64, queue_size=10)
    pos_pub4 = rospy.Publisher('/cont4/command', Float64, queue_size=10)
    pos_pub5 = rospy.Publisher('/cont5/command', Float64, queue_size=10)
    rospy.init_node('py_node', anonymous=True)
    rate = rospy.Rate(10)
    print(type(pos_pub1))
    pos_pub1.publish(0.1)
    rospy.loginfo(0.1)
    pos_pub2.publish(Float64(pos[1]))
    rospy.loginfo(Float64(pos[1]))
    pos_pub3.publish(Float64(pos[2]))
    rospy.loginfo(Float64(pos[2]))
    pos_pub4.publish(Float64(pos[3]))
    rospy.loginfo(Float64(pos[3]))
    pos_pub5.publish(Float64(pos[4]))
    rospy.loginfo(Float64(pos[4]))
    rate.sleep()
   

if __name__ == '__main__':
   
    try:
        cont()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass