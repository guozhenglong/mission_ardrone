#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import Header
from geometry_msgs.msg import Pose  # , Point, Vector3, Twist
import math


def pub_target():
    rospy.init_node('pub_target_pose')
    pub_tgt_1 = rospy.Publisher('/target1', Pose, queue_size=10)
    pub_tgt_2 = rospy.Publisher('/target2', Pose, queue_size=10)
    pub_tgt_3 = rospy.Publisher('/target3', Pose, queue_size=10)
    Hz = 20
    r = rospy.Rate(Hz)
    msg_tgt_1 = Pose()
    msg_tgt_2 = Pose()
    msg_tgt_3 = Pose()
    q = 0
    r = 1.0
    x0 = 0.0
    y0 = 0.0
    dq = 5
    T = 0.2
    h = 1.2
    ang = 0
    pi = 3.14
    v = dq / 180.0 * pi * r * Hz



    while not rospy.is_shutdown():
        x1 = x0 + r * math.cos(q * pi / 180)
        y1 = y0 + r * math.sin(q * pi / 180)

        x2 = x0 + r * math.cos((q + 120) * pi / 180)
        y2 = y0 + r * math.sin((q + 120) * pi / 180)

        x3 = x0 + r * math.cos((q + 240) * pi / 180)
        y3 = y0 + r * math.sin((q + 240) * pi / 180)


        msg_tgt_1.position.x = x1  # x
        msg_tgt_1.position.y = y1  # y
        msg_tgt_1.position.z = h # z
        msg_tgt_1.orientation.x = -v*math.sin(q/180*pi)  # vx
        msg_tgt_1.orientation.y = v*math.cos(q/180*pi)  # vy
        msg_tgt_1.orientation.z = 0.0  # vz
        msg_tgt_1.orientation.w = ang  # yaw

        msg_tgt_2.position.x = x2  # x
        msg_tgt_2.position.y = y2  # y
        msg_tgt_2.position.z = h  # z
        msg_tgt_2.orientation.x = -v*math.sin((q+120)/180*pi) # vx
        msg_tgt_2.orientation.y = v*math.cos((q+120)/180*pi)   # vy
        msg_tgt_2.orientation.z = 0.0  # vz
        msg_tgt_2.orientation.w = ang  # yaw

        msg_tgt_3.position.x = x3  # x
        msg_tgt_3.position.y = y3  # y
        msg_tgt_3.position.z = h  # z
        msg_tgt_3.orientation.x = -v*math.sin((q+240)/180*pi)  # vx
        msg_tgt_3.orientation.y = v*math.cos((q+240)/180*pi)   # vy
        msg_tgt_3.orientation.z = 0.0  # vz
        msg_tgt_3.orientation.w = ang  # yaw

        pub_tgt_1.publish(msg_tgt_1)
        pub_tgt_2.publish(msg_tgt_2)
        pub_tgt_3.publish(msg_tgt_3)

        q = q + dq
        r.sleep()



if __name__ == '__main__':
    try:
        pub_target()
    except rospy.ROSInterruptException:
        pass