#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from digit_interface import Digit
from cv_bridge import CvBridge
import cv2


class DualDigitRosNode:
    def __init__(self):
        rospy.init_node('gelsight_dual_digit_node', anonymous=True)

        # Parameters
        self.device_left_id = rospy.get_param('~device_left_id', 'D21237')
        self.device_right_id = rospy.get_param('~device_right_id', 'D21236')
        self.left_topic = rospy.get_param('~left_topic_name', '/digit/left/image_raw')
        self.right_topic = rospy.get_param('~right_topic_name', '/digit/right/image_raw')
        self.publish_rate = rospy.get_param('~publish_rate', 30)
        self.verbose = rospy.get_param('~verbose', 0)

        # Setup
        self.bridge = CvBridge()
        self.pub_left = rospy.Publisher(self.left_topic, Image, queue_size=10)
        self.pub_right = rospy.Publisher(self.right_topic, Image, queue_size=10)

        self.digit_left = Digit(self.device_left_id)
        self.digit_right = Digit(self.device_right_id)

        self.digit_left.connect()
        self.digit_right.connect()

        rospy.loginfo(f"Connected to DIGIT sensors: {self.device_left_id}, {self.device_right_id}")

    def capture_and_publish(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            try:
                frame_left = self.digit_left.get_frame()
                frame_right = self.digit_right.get_frame()
                timestamp = rospy.Time.now()

                msg_left = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
                msg_right = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')

                msg_left.header.stamp = timestamp
                msg_left.header.frame_id = "digit_left"
                msg_right.header.stamp = timestamp
                msg_right.header.frame_id = "digit_right"

                self.pub_left.publish(msg_left)
                self.pub_right.publish(msg_right)

                if self.verbose:
                    rospy.loginfo("Published images from both DIGIT sensors")

                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error capturing from DIGIT sensors: {e}")

    def cleanup(self):
        self.digit_left.disconnect()
        self.digit_right.disconnect()
        rospy.loginfo("Disconnected both DIGIT sensors")


def main():
    node = DualDigitRosNode()
    try:
        node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()
