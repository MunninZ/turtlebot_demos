#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('set_turtlebot_velocity')
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cmd_vel_msg = Twist()
    loop_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        cmd_vel_msg.linear.x = 0.15 # 0.15 m/s 
        cmd_vel_msg.angular.z = 1 # 1 rad/s
        cmd_vel_pub.publish(cmd_vel_msg)
        loop_rate.sleep()