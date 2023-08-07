#!/usr/bin/env python3

import math

import rospy
import tf
import numpy as np
import quaternion


if __name__ == '__main__':
    rospy.init_node("joint_positions")
    listener = tf.TransformListener()
    arm_id = rospy.get_param("joint_positions/arm_id")


    rate = rospy.Rate(10.0)
    frames = ['link_1_s', 'link_2_l', 'link_3_u', 'link_4_r', 'link_5_b', 'link_6_t', 'flange', 'tool0']

    while not rospy.is_shutdown():
       joint_positions = []
       joint_positions.append([0.4, -0.5, 0.3]) # To prevent 'valleys' in the curtain
       base_frame = f'{arm_id}/' + 'base_link'
       for frame in frames:
           frame = f'{arm_id}/' + frame
           try:
               (position, rot) = listener.lookupTransform(base_frame, frame, rospy.Time(0))
               joint_positions.append([position[0], position[1], position[2]])
               joint_positions.append([position[0] + 0.3, position[1], position[2]])
               joint_positions.append([position[0] - 0.3, position[1], position[2]])
               joint_positions.append([position[0], position[1], position[2] + 0.3])
               joint_positions.append([position[0], position[1], position[2] - 0.3])
               joint_positions.append([position[0], position[1] + 0.3, position[2]])
               joint_positions.append([position[0], position[1] - 0.3, position[2]])
               joint_positions.append([position[0] + 0.3, position[1], position[2] + 0.3])
               joint_positions.append([position[0] - 0.3, position[1], position[2] + 0.3])
               joint_positions.append([position[0] + 0.3, position[1], position[2] - 0.3])
               joint_positions.append([position[0] - 0.3, position[1], position[2] - 0.3])

           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               continue
    
       rospy.set_param('joint_positions', joint_positions)
       rate.sleep()
