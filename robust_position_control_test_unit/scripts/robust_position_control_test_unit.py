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
from std_msgs.msg import String

if __name__ == '__main__':
    '''
    Initilize node and string type publish topic at the loop rate.
    '''
    rospy.init_node('robust_position_control_test_unit', anonymous=False)
    rospy.loginfo("robust_position_control_test_unit is now running")

    joint_position_pub = rospy.Publisher("/robust_position_controller/component_input", JointPositions, queue_size=100)

    joint_position_string_pub = rospy.Publisher("/robust_position_controller/event_in", String, queue_size=100)

    joint_position_msg = JointPositions()

    string_msg= String()
    string_msg.data = "e_start"
   

    joint_names = ["arm_joint_1" , "arm_joint_5"]

    joint_value_=[0.2, 0.2]

    
    for i in range (0,len(joint_names)):
        joint_value_msg = JointValue()

        joint_value_msg.joint_uri = joint_names[i]
        
        joint_value_msg.value = joint_value_[i]

        joint_value_msg.unit="rad"

        joint_position_msg.positions.append(joint_value_msg)
    loop_rate = rospy.Rate(10)
        
    while not rospy.is_shutdown():
        joint_position_pub.publish(joint_position_msg)
        rospy.sleep(0.5)
        joint_position_string_pub.publish(string_msg)
        loop_rate.sleep()

    
    

    
