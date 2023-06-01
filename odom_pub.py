#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from math import sin, cos

class JointStateToOdom:
    def __init__(self):
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_msg = Odometry()
        self.wheel_base = 0.5380
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_theta = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_time = rospy.Time.now()

    def joint_states_callback(self, joint_states_msg):
        delta_left = joint_states_msg.position[0] - self.last_left_pos
        delta_right = joint_states_msg.position[1] - self.last_right_pos
        delta_theta = (delta_right - delta_left) / self.wheel_base
        delta_s = (delta_left + delta_right) / 2.0
        theta = self.last_theta + delta_theta/2.0
        x = self.last_x + delta_s * cos(theta)
        y = self.last_y + delta_s * sin(theta)
        vx = delta_s / (rospy.Time.now() - self.last_time).to_sec()
        vth = delta_theta / (rospy.Time.now() - self.last_time).to_sec()

        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom_frame"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = sin(theta/2.0)
        self.odom_msg.pose.pose.orientation.w = cos(theta/2.0)
        self.odom_msg.twist.twist.linear.x = vx
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.angular.z = vth
        self.odom_pub.publish(self.odom_msg)

        # Update last_* variables for next callback
        self.last_left_pos = joint_states_msg.position[0]
        self.last_right_pos = joint_states_msg.position[1]
        self.last_theta = theta
        self.last_x = x
        self.last_y = y
        self.last_time = rospy.Time.now()
        
if __name__ == '__main__':
    rospy.init_node('odom_pub')
    print('started odom publisher node')
    joint_state_to_odom = JointStateToOdom()
    rospy.spin()
