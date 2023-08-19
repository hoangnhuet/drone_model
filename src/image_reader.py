import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # Convert the ROS Image message to an OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image data here (optional)
        # For example, you can display the image using cv2.imshow()
        cv2.imshow("Image Viewer", cv_image)
        cv2.waitKey(1)

        # Print the image data (optional)
        rospy.loginfo("Image data:\n%s", msg)

    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

def main():
    rospy.init_node('image_reader_node', anonymous=True)
    image_topic = "/front_cam/camera/image"  
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

