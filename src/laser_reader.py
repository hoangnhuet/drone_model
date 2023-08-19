#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    try:
        # Process the laser scan data here
        # For example, you can access the range data and angles using:
        ranges = msg.ranges
        angles = msg.angle_min + msg.angle_increment * (len(ranges) - 1)
        # Do whatever you want with the laser scan data
        rospy.loginfo("Received Laser Scan Data:")
        rospy.loginfo("Ranges: %s", ranges)
        rospy.loginfo("Angles: %s", angles)

    except Exception as e:
        rospy.logerr("Error processing laser scan data: %s", str(e))

def main():
    rospy.init_node('laser_reader_node', anonymous=True)
    laser_topic = "/scan"
    rospy.Subscriber(laser_topic, LaserScan, laser_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

