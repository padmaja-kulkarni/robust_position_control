#!/usr/bin/env python
"""
This script is a simple python node which imports
source code of std_messages_examples.
"""
#-*- encoding: utf-8 -*-
__author__ = 'padmaja_kulkarni'

import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue

if __name__ == '__main__':
    '''
    Initilize node and string type publish topic at the loop rate.
    '''
    rospy.init_node('robust_position_control_test_unit', anonymous=False)
    rospy.loginfo("robust_position_control_test_unit is now running")

    joint_position_pub = rospy.Publisher("/robust_position_controller/component_input", JointPositions, queue_size=100)

    joint_position_msg = JointPositions()
   

    joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3"]

    joint_value_=[0.1,0.2,0.3]

    
    for i in range (0,3):
        joint_value_msg = JointValue()

        joint_value_msg.joint_uri = joint_names[i]
        
        joint_value_msg.value = joint_value_[i]

        joint_value_msg.unit="rad"

        joint_position_msg.positions.append(joint_value_msg)
    loop_rate = rospy.Rate(10)
        
    while not rospy.is_shutdown():
     joint_position_pub.publish(joint_position_msg)
     loop_rate.sleep()

    
    

    
