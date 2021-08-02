#!/usr/bin/env python3

import rospy
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, ConfigureControlModeResponse
from iiwa_msgs.msg import CartesianImpedanceControlMode, CartesianQuantity,JointImpedanceControlMode,DesiredForceControlMode,SinePatternControlMode,CartesianControlModeLimits

rospy.init_node('set_CartesianImpedance', anonymous=False)

rospy.wait_for_service("/iiwa/configuration/ConfigureControlMode")

control_mode = 2 #for Cartesian Impedance control mode
joint_impedance = JointImpedanceControlMode()
cartesian_impedance = CartesianImpedanceControlMode()
desired_force = DesiredForceControlMode()
sine_pattern = SinePatternControlMode()
limits = CartesianControlModeLimits()

#stiffness
stiffness = CartesianQuantity()
stiffness.x =1000
stiffness.y =1000
stiffness.z =250
stiffness.a =150
stiffness.b =150
stiffness.c =10
cartesian_impedance.cartesian_stiffness = stiffness
#damping
damping = CartesianQuantity()

damping.x = 0.1
damping.y = 0.5
damping.z = 1.0
damping.a = 0.5
damping.b = 0.5
damping.c = 0.5

cartesian_impedance.cartesian_damping = damping

#null space stiffness
cartesian_impedance.nullspace_stiffness  = 250
#null space damping
cartesian_impedance.nullspace_damping = 0.7


try:
    change_control = rospy.ServiceProxy("/iiwa/configuration/ConfigureControlMode", ConfigureControlMode)
    resp = change_control(control_mode, joint_impedance, cartesian_impedance, desired_force,sine_pattern,limits)
    print("Response : ", resp.success)
except (rospy.ServiceException):
    print ("Service call failed ") 
