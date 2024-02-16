#!/usr/bin/env python3

import rospy
from docking_control.srv import DockingControll
from geometry_msgs.msg import Twist
from custom_opencv import detection
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
import cv2
import numpy as np

class Docking_Aruco:

    def __init__(self):

        self.bridge = CvBridge()
        self.start = False
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback_image)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback_lidar)
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.service = rospy.Service('docking_aruco_service', DockingControll, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_msg = Twist()
        self.angular_p = rospy.get_param("navigation/angular_p")
        self.radius_threshold = rospy.get_param("navigation/radius_threshold")
        self.theta_precision = rospy.get_param("navigation/theta_precision")
        self.linear_p = rospy.get_param("navigation/linear_p")
        
        self.linear_velocity = 0.1
        self.angular_velocity = 0.1
        self.start = False
        self.detect = detection()
        self.detect.T = 3
        self.lt = ""
        self.at = ""
        self.scan_ranges = None 
        

    def move(self, linear, angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        rospy.loginfo("linear velocity " + str(linear))
        rospy.loginfo("angular velocity " + str(angular))

        self.pub.publish(self.velocity_msg)
        
        
    def callback_image(self, data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.start:
                self.control_loop()
        except CvBridgeError as e:
            print(e)

    def callback_lidar(self, data):
        self.scan_ranges = np.array(data.ranges)

    def callback(self, req):
        #response = 8
        if not self.start:
            self.start = True
        return 1
        

    def direction(self, markerID):

        if markerID == 1:
            return 0, "Parked", "ID - 1"
        elif markerID == 2:
            return -0.3, "Turning Right", "ID - 2"
        else:
            return 0.3, "", "Turning Left" , "ID - 3"
            
            
    def move_closer_to_marker_with_clearance(self):
    # Move closer to the marker while maintaining a 12 cm clearance
        desired_distance = 18  # cm
        current_distance = self.Result[2]  # Distance to marker
        move_distance = current_distance - desired_distance
        linear_velocity = move_distance * 0.001  # Adjust linear velocity to move closer
        self.move(linear_velocity, 0)  
         
    def stop_moving_closer_to_marker(self):
        # Stop moving closer to the marker
        self.move(0, 0)     


    def control_loop(self):
        self.Result = self.detect.aruco_detection(self.cv1_image)

        x_length = self.Result[0].shape[0]
        x_length = x_length + 40
        rospy.loginfo("linear_p" +str(self.linear_p))

        # if self.detect.markerID1 != 0 and self.detect.center is not None and self.detect.radius1 is not None:
        #     aruco_position = self.Result[1][0]
        #     self.theta_error = int(x_length) / 2 - aruco_position

        #     rospy.loginfo("aruco_postion" + str(aruco_position))
        #     rospy.loginfo("x lenght" + str(x_length))
        #     rospy.loginfo("radius" + str(self.Result[2]))
        #     rospy.loginfo("theta error " + str(self.theta_error))
            
        #     condition_for_moving_left = self.theta_error > 0 and (self.theta_precision > abs(self.theta_error))
        #     condition_for_moving_right = self.theta_error < 0 and (self.theta_precision > abs(self.theta_error))
            
        #     if self.scan_ranges is not None and min(self.scan_ranges) < self.radius_threshold:
        #         # Obstacle detected, stop moving
        #         self.move(0.0001, 0)
        #         self.at = "Obstacle Detected"
        #         self.lt = "Stop"
        #     else:
        #         if self.Result[2] < self.radius_threshold - 10:
        #             self.detect.T = 3
        #             if self.theta_error > 0 and (self.theta_precision < abs(self.theta_error)):
        #                 self.move(self.linear_p * (self.radius_threshold - self.Result[2]),
        #                         self.angular_p * self.theta_error )
        #                 self.at = " <- LEFT"
        #                 self.lt = "Move Forward"
        #                 print("left")
        #             elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
        #                 self.move(self.linear_p * (self.radius_threshold - self.Result[2]),
        #                         self.angular_p * self.theta_error)
        #                 self.at = "  RIGHT->"
        #                 self.lt = "Move Forward"
        #                 print("right")
        #             else:
        #                 self.move(self.linear_p * (self.radius_threshold - self.Result[2]), 0)
        #                 self.at = " CENTER "
        #                 self.lt = "Move Forward"
        #                 print("straight")
        #         else:
        #             if condition_for_moving_left:
        #                 self.move(0, self.angular_p * self.theta_error)
        #                 self.at = "<- LEFT"
        #                 self.lt = "Stop"
        #             elif condition_for_moving_right:
        #                 self.move(0, self.angular_p * self.theta_error)
        #                 self.at = " RIGHT->"
        #                 self.lt = "Stop"
        #             else:
        #                 angular = self.direction(self.detect.markerID1)
        #                 self.lt = angular[2]
        #                 self.at = angular[1]
        #                 self.move(0, angular[0])
        #                 if angular[1] == "Parked" and not condition_for_moving_left and not condition_for_moving_right:
        #                     print("PARKED")
        #                     exit()
            
        #     # # Check the distance to the marker and move closer if it's further than 12 cm
        #     if self.Result[2] > 18:
        #         self.move_closer_to_marker_with_clearance()
        #     #     if condition_for_moving_left:
        #     #         self.move(0, self.angular_p * self.theta_error)
        #     #         self.at = "<- LEFT"
        #     #         self.lt = "Stop"
        #     #     elif condition_for_moving_right:
        #     #         self.move(0, self.angular_p * self.theta_error)
        #     #         self.at = " RIGHT->"
        #     #         self.lt = "Stop"
        #     #     else:
        #     #         angular = self.direction(self.detect.markerID1)
        #     #         self.lt = angular[2]
        #     #         self.at = angular[1]
        #     #         self.move(0, angular[0])
        #     #     if angular[1] == "Parked" and not condition_for_moving_left and not condition_for_moving_right:
        #     #         print("PARKED")
        #     #         exit()
        #     else:
        #         self.stop_moving_closer_to_marker()
        if self.Result is not None and self.Result[0] is not None and self.detect.markerID1 != 0 and self.detect.center is not None and self.detect.radius1 is not None:
            print(self.Result[1][0])
            aruco_position = self.Result[1][0]  # X position of the AR marker
            image_width = self.cv1_image.shape[1]  # Width of the image frame

            # Calculate the difference between the center of the image and the AR marker position
            offset = (image_width // 2) - aruco_position

            # Adjust the angular velocity based on the offset
            max_angular_velocity = 0.2  # Maximum angular velocity (adjust as needed)
            angular_velocity = np.clip(0.01 * offset, -max_angular_velocity, max_angular_velocity)  # Adjust this coefficient as needed

            # Publish the velocity command to turn the robot
            self.move(0, angular_velocity)

            # Check lidar distance to AR marker
            if self.scan_ranges is not None:
                min_distance = np.min(self.scan_ranges)
                desired_distance = 0.5  # Adjust this desired distance as needed
                if min_distance > desired_distance:
                    # Move forward
                    self.move(0.1, 0)
                else:
                    # Stop
                    self.move(0, 0)
                    cv2.destroyAllWindows()
                    self.start = False
                    self.detect.markerID1 = 0
                    self.detect.center = None
                    self.detect.radius1 = None

        else:
            self.move(0, 0.4)
            self.at = "Finding Aruco"

        
        
        if self.start:
            cv2.putText(self.Result[4], self.lt, (300, 800), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(self.Result[4], self.at, (350, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.imshow("Frame", self.Result[4])
            cv2.waitKey(1)


    def run(self):
        s = rospy.Service('docking_aruco_service', DockingControll, self.callback)
        #rospy.spin()
        

if __name__ == '__main__':
    rospy.init_node('docking_aruco')
    rospy.loginfo("Docking_aruco_live")
    aruco = Docking_Aruco()
    try:
        rospy.spin()
    except:
        pass    
