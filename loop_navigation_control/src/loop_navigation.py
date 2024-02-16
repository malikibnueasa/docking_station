#!/usr/bin/env python3

import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from time import sleep
from std_msgs.msg import Bool
from docking_control.srv import DockingControll


class MoveNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.is_charging_flag = False
        self.counter = 0
        self.checker = None

    def go_to(self, x, y, z):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if wait:
            sleep(1)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        return self.client.get_result()

    def home_position(self, loc):
        if loc == 0:
            rospy.loginfo("Heading to First Point")
            self.go_to(0.3327469671548591, -0.07669310109807612, -0.34303521623827754)
        elif loc == 1:
            rospy.loginfo("Heading to Second Point")
            self.go_to(1.8487239882015116, 1.9633121738718773, -0.6458175417427996)
        elif loc == 2:
            rospy.loginfo("Heading to Third Point")
            self.go_to(2.57994547476032, -1.925609675033026, -1.5745951272044802)
        elif loc == 3:
            rospy.loginfo("Heading to Fourth Point")
            self.go_to(4.3941970305451905, -0.23586030441848677, -0.10364319482621749)
        elif loc == 4:
            rospy.loginfo("Heading to Charging Docking Station")
            self.go_to(0.003656124611378742, 1.0840664377839129, 1.5887551547854604)

    def cancel_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled")
    
    def docking_station_navigation(self):
        
        self.cancel_goal()
        self.home_position(4)
        self.service_call_docking()

    def service_call_docking(self):
        rospy.wait_for_service('docking_service')
        try:
            rospy.loginfo("Aligning Robot to Docking Station")
            server_proxy = rospy.ServiceProxy('docking_service', DockingControll)
            response = server_proxy()
            if response.value == 1:
                rospy.loginfo("Fully")
                self.is_charging_flag = False
                self.checker = None
            else:
                rospy.loginfo("Docking failed")
                self.is_charging_flag = True

        except rospy.ServiceException as e:
            print('Servide call failed: ')    

    def loop_navigation(self):
        if self.counter == 0:
            self.home_position(0)
            self.counter = 1
        elif self.counter == 1:
            self.home_position(1)
            self.counter = 2
        elif self.counter == 2:
            self.home_position(2)
            self.counter = 3
        elif self.counter == 3:
            self.home_position(3)
            self.counter = 0

    def flag_check(self, msg):
        self.is_charging_flag = msg.data
        if self.checker != msg.data and self.is_charging_flag:
            self.cancel_goal()
        else:
            pass
        self.checker = msg.data

    def run(self):
        try:
            rospy.init_node('navigation_logical_node')
            rospy.Subscriber('battery_low_power_status', Bool, self.flag_check)
            while not rospy.is_shutdown():
                while not self.is_charging_flag:
                    self.loop_navigation()
                while self.is_charging_flag:
                    self.docking_station_navigation()
        except rospy.ROSInterruptException:
            # Exception thrown when rospy.is_shutdown() is True
            self.cancel_goal()
            rospy.loginfo("Shutting down...")
        except KeyboardInterrupt:
            # Exception thrown when the user presses Ctrl+C
            self.cancel_goal()
            rospy.loginfo("Keyboard interrupt received. Shutting down...")
        except Exception as e:
            # Catch any other exceptions and handle them
            self.cancel_goal()
            rospy.logerr("An error occurred: %s", str(e))



if __name__ == '__main__':
    rospy.init_node('navigation_logical_node')  
    move_node = MoveNode()
    move_node.run()

