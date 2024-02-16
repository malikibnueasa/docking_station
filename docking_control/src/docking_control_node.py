#!/usr/bin/env python3

import rospy
from docking_control.srv import DockingControll
from std_msgs.msg import Bool
import time
import threading

class DockingService:
    def __init__(self):
        self.publisher_low_power_status = rospy.Publisher('battery_charging_mode', Bool, queue_size=10)
        self.battery_low_power_status = False  # Initialize to False
        self.lock = threading.Lock()

    def start_charging(self):
        with self.lock:
            for _ in range(10):
                self.publisher_low_power_status.publish(True)
                time.sleep(1)  # Add delay between publishes

    def waiting_for_charging(self):
        rospy.loginfo("Waiting for battery_low_power_status...")
        while self.battery_low_power_status:
            time.sleep(1)
        rospy.loginfo("Waiting for battery_low_power_status...")
        return True

    def flag_check(self, msg):
        
        self.battery_low_power_status = msg.data
        
    def callback(self, req):
        # Assuming you'll do alignment first here
        rospy.loginfo("Aligning the robot to the docking station...")
        self.aruco_control()
        #start charging the battery 
        self.start_charging()
        #wait to complete chargings
        success = self.waiting_for_charging()

        # Return appropriate flag
        if success:
            self.publisher_low_power_status.publish(False)
            return 1
        else:
            return 0

    def aruco_control(self):
        rospy.wait_for_service('docking_aruco_service')
        try:
            server_proxy_aruco = rospy.ServiceProxy('docking_aruco_service', DockingControll)
            response = server_proxy_aruco()
            print(response.value)
        except rospy.ServiceException as e:
            print('Servide call failed: ')

    def run(self):
        rospy.Subscriber('battery_low_power_status', Bool, self.flag_check)
        s = rospy.Service('docking_service', DockingControll, self.callback)
        rospy.loginfo("Docking_control live")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('docking_control')
    docking = DockingService()
    docking.run()
