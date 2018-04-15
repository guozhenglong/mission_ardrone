#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, TwistStamped
from vps_driver.msg import vps_navdata
pub_vel = rospy.Publisher('/vel_vps', PointStamped, queue_size=100)
pub_pose = rospy.Publisher('/pose_vps', TwistStamped, queue_size=100)





def vps_nav_CB(data):
    vel_ins = PointStamped()
    pose_ins = TwistStamped()
    vel_ins.header = Header()
    pose_ins.header = Header()
    pose_ins.twist.linear.x = data.X_e
    pose_ins.twist.linear.y = data.Y_e
    pose_ins.twist.linear.z = data.Z_e
    pose_ins.twist.angular.x = 0.0
    pose_ins.twist.angular.y = 0.0
    pose_ins.twist.angular.z = data.yaw_e
    vel_ins.point.x = data.Vx_e
    vel_ins.point.y = data.Vy_e
    vel_ins.point.z = data.Vz_e
    pub_vel(vel_ins)
    pub_pose(pose_ins)
    
    rospy.loginfo(" Completely transform to standard ROS messages.")

def trans_vps_navdata():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('trans_vpsnavdata', anonymous=True)
    rospy.Subscriber("/vpsnavdata", vps_navdata, vps_nav_CB)
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    trans_vps_navdata()