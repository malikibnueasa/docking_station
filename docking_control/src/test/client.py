#!/usr/bin/env python3

# import rospy
# from docking_control.srv import DockingControll

# def add(x,y):
#     rospy.wait_for_service('docking_service')
#     try:
#         server_proxy = rospy.ServiceProxy('docking_service', DockingControll)
#         response = server_proxy(x,y)
#         print(response.value)
#     except rospy.ServiceException as e:
#         print('Servide call failed: ')

# if __name__ == '__main__':
#     rospy.init_node('docking_client')
    
#     x,y = 3,5
#     add(x,y)


import rospy
from docking_control.srv import DockingControll

def add():
    rospy.wait_for_service('docking_service_test')
    try:
        server_proxy = rospy.ServiceProxy('docking_service_test', DockingControll)
        response = server_proxy()
        print(response.value)
    except rospy.ServiceException as e:
        print('Servide call failed: ')

if __name__ == '__main__':
    rospy.init_node('docking_client')
    
    add()