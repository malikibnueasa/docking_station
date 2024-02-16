#!/usr/bin/env python3

# import rospy
# from docking_control.srv import DockingControll

# def callback(msg):
#     response = (msg.a+msg.b)
     
#     return response
    
# if __name__ == '__main__':
#     rospy.init_node('docking_control')
#     s = rospy.Service('docking_service', DockingControll, callback)
#     rospy.loginfo("Docking_controll_live")
#     rospy.spin()


import rospy
from docking_control.srv import DockingControll

def callback(msg):
    response = 8
     
    return response
    
if __name__ == '__main__':
    rospy.init_node('docking_control_test')
    s = rospy.Service('docking_service_test', DockingControll, callback)
    rospy.loginfo("Docking_controll_live")
    rospy.spin()
