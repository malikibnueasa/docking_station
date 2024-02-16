#!/usr/bin/env python3
from std_msgs.msg import Bool, String


import rospy
rospy.init_node('test')
pub = rospy.Publisher('battery_charging_mode', Bool, queue_size=10)

while True:
    if (int(input("True : 1 and False : 2  :: "))) == 1:
        print("true")
        pub.publish(True)
    else:
        print("false")
        pub.publish(False)