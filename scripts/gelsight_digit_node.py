import rospy
from sensor_msgs.msg import Image
import cv2
from digit_interface import Digit
from cv_bridge import CvBridge


class DigitRosNode:
    def __init__(self):
        rospy.init_node('gelsight_digit_node', anonymous=True)

        # ROS parameters
        self.device_id = rospy.get_param('~device_id', 'D21237')  # Replace with your Digit device ID
        self.topic_name = rospy.get_param('~topic_name', '/digit/image_raw')
        self.publish_rate = rospy.get_param('~publish_rate', 30)

        # Publisher setup
        self.image_publisher = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.bridge = CvBridge()

        # Initialize Digit Sensor
        self.digit_sensor = Digit(self.device_id)
        self.digit_sensor.connect()

        rospy.loginfo(f"Connected to DIGIT sensor with ID {self.device_id}")

    def capture_and_publish(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                # Capture image from the Digit sensor
                frame = self.digit_sensor.get_frame()

                # Convert to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

                # Publish the image
                self.image_publisher.publish(ros_image)

                # Log publishing info
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "digit"
                rospy.loginfo("Publishing image from DIGIT sensor")

                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error capturing image from DIGIT sensor: {e}")

    def cleanup(self):
        self.digit_sensor.disconnect()
        rospy.loginfo("Disconnected from DIGIT sensor")


def main():
    node = DigitRosNode()
    try:
        node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()
