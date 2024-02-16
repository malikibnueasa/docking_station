#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, String

initial_battery_percentage = 100
battery_life = 100
cutoff_battery_percentage = 30
charging_time = 20

class BatteryManager:
    def __init__(self,initial_battery_percentage, battery_life, cutoff_battery_percentage, charging_time):
        self.current_battery_percentage = initial_battery_percentage
        self.battery_life = battery_life
        self.cutoff_battery_percentage = cutoff_battery_percentage
        self.charging_time = charging_time
        self.is_charging_flag = False
        self.charging_rate = (100 / self.charging_time)
        self.discharging_rate = (100 / self.battery_life)
        self.publisher_low_power_status = rospy.Publisher('battery_low_power_status', Bool, queue_size=10)
        self.publisher_battery_manager_log = rospy.Publisher('battery_status_logger', String, queue_size=10)

    def flag_check(self, msg):
        
        self.is_charging_flag = msg.data
        

    def run(self):
        rospy.Subscriber('battery_charging_mode', Bool, self.flag_check)
        while not rospy.is_shutdown():
            if self.is_charging_flag:
                #charging
                self.charging()
            else:
                #discharging
                self.discharge()


    def discharge(self):
        rospy.loginfo("Switched to discharging mode.")
        self.publisher_battery_manager_log.publish("Switched to discharging mode.")
        while self.is_charging_flag == False:
            self.current_battery_percentage -= self.discharging_rate
            if self.current_battery_percentage <= 0:
                self.current_battery_percentage = 0
            rospy.loginfo("Battery Percentage : {:.2f}%".format(self.current_battery_percentage))
            self.publisher_battery_manager_log.publish("Battery Percentage : {:.2f}%".format(self.current_battery_percentage))
            time.sleep(1)
            if self.current_battery_percentage <= self.cutoff_battery_percentage and self.current_battery_percentage > 0:
                self.publisher_low_power_status.publish(True)
                time.sleep(2)
                rospy.loginfo("Battery is low, Searching for Docking station.")
                self.publisher_battery_manager_log.publish("Battery is low, Searching for Docking station.")
            elif self.current_battery_percentage <= 0:
                #enable power of command
                rospy.loginfo("Battery is too low, Power off")
                self.publisher_battery_manager_log.publish("Battery is too low, Power off")
                self.is_charging_flag = None




    def charging(self):
        
        rospy.loginfo("Switched to charging mode.")
        self.publisher_battery_manager_log.publish("Switched to charging mode.")
        while self.current_battery_percentage < 100 and self.is_charging_flag:
            self.current_battery_percentage += self.charging_rate
            if self.current_battery_percentage >= 100:
                self.current_battery_percentage = 100
            #log battery percentage. print(f"Battery percentage: {current_percentage:.2f}%")
            rospy.loginfo("Charging!! Battery Percentage : {:.2f}%".format(self.current_battery_percentage))
            self.publisher_battery_manager_log.publish("Charging!! Battery Percentage : {:.2f}%".format(self.current_battery_percentage))
            time.sleep(1)
        if self.current_battery_percentage == 100:
            rospy.loginfo("Fully Charged!!")
            self.publisher_low_power_status.publish(False)
            self.publisher_battery_manager_log.publish("Fully Charged!!") 
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('battery_manager_node')
    battery = BatteryManager(initial_battery_percentage, battery_life, cutoff_battery_percentage, charging_time)
    battery.run()
