#!/usr/bin/python3
import mavros.command
import rospy
from mavros_msgs.srv import SetMode
import mavros

def arm():
    mavros.set_namespace()
    mavros.command.arming(True)

def set_stabilized_mode():
    set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/set_mode')
    set_mode(0, 'STABILIZE')

def set_depth_mode():
    set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/set_mode')
    set_mode(0, 'ALT_HOLD')

inp = input("Enter the mode you want to set: ")

arm()
print("The pixhawk has been armed")

if inp == "STABILIZE":
    set_stabilized_mode()
    print("Stabilize function called")
elif inp == "DEPTH HOLD":
    set_depth_mode()
    print("Depth hold function called")

