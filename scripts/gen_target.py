#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import Header
from geometry_msgs.msg import Pose #, Point, Vector3, Twist

def pub_target():
    rospy.init_node('pub_target_pose')
    pub_tgt_1 = rospy.Publisher('/target1', Pose, queue_size=10)
    pub_tgt_2 = rospy.Publisher('/target2', Pose, queue_size=10)
    pub_tgt_3 = rospy.Publisher('/target3', Pose, queue_size=10)
    rate = rospy.Rate(20) 

    while not rospy.is_shutdown():
        msg_tgt_1 = Pose()
        msg_tgt_2 = Pose()
        msg_tgt_3 = Pose()

        msg_tgt_1.position.x = 1.0 # x
        msg_tgt_1.position.y = 0.0 # y
        msg_tgt_1.position.z = 1.0 # z
        msg_tgt_1.orientation.x = 0.0   # vx
        msg_tgt_1.orientation.y = 0.0   # vy
        msg_tgt_1.orientation.z = 0.0   # vz
        msg_tgt_1.orientation.w = 0.0   # yaw    
                
        msg_tgt_2.position.x = 0.0 # x
        msg_tgt_2.position.y = 1.0 # y
        msg_tgt_2.position.z = 1.0 # z
        msg_tgt_2.orientation.x = 0.0   # vx
        msg_tgt_2.orientation.y = 0.0   # vy
        msg_tgt_2.orientation.z = 0.0   # vz
        msg_tgt_2.orientation.w = 0.0   # yaw   

        msg_tgt_3.position.x = 1.0 # x
        msg_tgt_3.position.y = 1.0 # y
        msg_tgt_3.position.z = 1.0 # z
        msg_tgt_3.orientation.x = 0.0   # vx
        msg_tgt_3.orientation.y = 0.0   # vy
        msg_tgt_3.orientation.z = 0.0   # vz
        msg_tgt_3.orientation.w = 0.0   # yaw

        pub_tgt_1.publish(msg_tgt_1)
        pub_tgt_2.publish(msg_tgt_2)
        pub_tgt_3.publish(msg_tgt_3)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_target()
    except rospy.ROSInterruptException: pass