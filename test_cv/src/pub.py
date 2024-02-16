#!/usr/bin/env python3

# # Import the necessary libraries
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class ImagePublisher:
#     """
#     Create an ImagePublisher class.
#     """

#     def _init_(self):
#         """
#         Class constructor to set up the node
#         """
#         # Initialize the ROS node
#         rospy.init_node('image_publisher')

#         # Create the publisher. This publisher will publish an Image
#         # to the video_frames topic. The queue size is 10 messages.
#         self.publisher = rospy.Publisher('video_frames', Image, queue_size=10)

#         # We will publish a message every 0.1 seconds
#         self.timer_period = rospy.Duration(0.1)

#         # Create the timer
#         self.timer = rospy.Timer(self.timer_period, self.timer_callback)

#         # Create a VideoCapture object
#         # The argument '0' gets the default webcam.
#         self.cap = cv2.VideoCapture(2)

#         # Used to convert between ROS and OpenCV images
#         self.br = CvBridge()

#     def timer_callback(self, event):
#         """
#         Callback function.
#         This function gets called every 0.1 seconds.
#         """
#         # Capture frame-by-frame
#         # This method returns True/False as well
#         # as the video frame.
#         ret, frame = self.cap.read()

#         if ret:
#             # Publish the image.
#             # The 'cv2_to_imgmsg' method converts an OpenCV
#             # image to a ROS image message
#             self.publisher.publish(self.br.cv2_to_imgmsg(frame))

#         # Display the message on the console
#         rospy.loginfo('Publishing video frame')

# def main():
#     # Create the ImagePublisher object
#     image_publisher = ImagePublisher()

#     # Keep the program running until a shutdown signal is received
#     rospy.spin()

# if __name__ == '_main_':
#     main()

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher:
    def _init_(self):
        rospy.init_node('image_publisher')
        self.publisher = rospy.Publisher('video_frames', Image, queue_size=10)
        self.timer_period = rospy.Duration(0.1)
        self.timer = rospy.Timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        # Check if VideoCapture initialization was successful
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera!")
            rospy.signal_shutdown("Failed to open camera")

    def timer_callback(self, event):
        ret, frame = self.cap.read()
        if ret:
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))
        rospy.loginfo('Publishing video frame')

def main():
    image_publisher = ImagePublisher()
    rospy.spin()

if __name__ == '_main_':
    main()