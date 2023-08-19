#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def control_robot_vx_1():
    rospy.init_node('control_robot')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    # Wait for publishers to connect
    rospy.sleep(1)

    # Takeoff
    takeoff_pub.publish(Empty())
    rospy.sleep(2)  # Wait for the drone to takeoff

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.0  # vx = 1
        vel_msg.linear.y = 0.0  # vy = 0
        vel_msg.linear.z = 1.0  # vz = 0
        vel_pub.publish(vel_msg)
        rate.sleep()

    # Land before the node exits
    land_pub.publish(Empty())
    rospy.sleep(2)  # Wait for the drone to land

if __name__ == "__main__":
    try:
        control_robot_vx_1()
    except rospy.ROSInterruptException:
        pass

