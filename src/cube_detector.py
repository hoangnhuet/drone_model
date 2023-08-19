#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class CubeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.cube_detected = False
        self.cube_center = None
        self.ranges = None
        self.angles = None

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform image processing to detect the cube
            # Replace the following code with your cube detection logic
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = (0, 100, 100)
            upper_red = (10, 255, 255)
            mask = cv2.inRange(hsv_image, lower_red, upper_red)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.cube_detected = False
            self.cube_center = None

            for contour in contours:
                if cv2.contourArea(contour) > 100:  # Set a suitable area threshold
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate the center of the cube (replace this with more accurate calculation if needed)
                    self.cube_center = (x + w // 2, y + h // 2)
                    self.cube_detected = True

            # Display the image with bounding boxes
            cv2.imshow("Image Viewer", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def laser_callback(self, msg):
        try:
            # Process the laser scan data here
            # For example, you can access the range data and angles using:
            self.ranges = msg.ranges
            self.angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
            # Do whatever you want with the laser scan data

        except Exception as e:
            rospy.logerr("Error processing laser scan data: %s", str(e))

    def main(self):
        rospy.init_node('cube_detector_node', anonymous=True)
        image_topic = "/front_cam/camera/image"  # Replace with the actual image topic
        rospy.Subscriber(image_topic, Image, self.image_callback)

        laser_topic = "/scan"
        rospy.Subscriber(laser_topic, LaserScan, self.laser_callback)

        rate = rospy.Rate(10)  # Adjust the rate as needed

        while not rospy.is_shutdown():
            if self.cube_detected:
                # Calculate the position of the cube using the detected center and laser scan data
                # You can use trigonometry and transformation between camera and laser frames
                # to estimate the 3D position of the cube relative to the robot

                rospy.loginfo("Cube detected at position: %s", self.cube_center)
                # Example: If the cube is at (x, y) in camera frame and at a distance d in laser frame:
                # x_laser = d * np.cos(self.angles)  # Calculate the x coordinate in the laser frame
                # y_laser = d * np.sin(self.angles)  # Calculate the y coordinate in the laser frame

            rate.sleep()

if __name__ == '__main__':
    try:
        detector = CubeDetector()
        detector.main()
    except rospy.ROSInterruptException:
        pass

