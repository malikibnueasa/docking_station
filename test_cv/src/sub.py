#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def _init_(self):
        """
        Class constructor to set up the node
        """
        # Initiate the ROS node
        rospy.init_node('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = rospy.Subscriber(
            'video_frames', 
            Image, 
            self.listener_callback, 
            queue_size=10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        rospy.loginfo('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Display image
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main():
    # Create the ImageSubscriber object
    image_subscriber = ImageSubscriber()

    # Keep the program running until a shutdown signal is received
    rospy.spin()

if __name__ == '_main_':
    main()