
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <algorithm>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
typedef std::vector<geometry_msgs::Twist>::iterator VecMovIt;

#ifndef CLAMP
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#endif
//example: ctrl_twist_.linear.x = CLAMP(ctrl_twist_.linear.x, -1.0, 1.0);

using namespace std;

namespace Mul_Drone_Ctrl
{
class drone_ctrl
{
  public:
    drone_ctrl(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    //void FillTargetList();
    void GenCmd(geometry_msgs::Twist &cmd,
                const geometry_msgs::Pose &goal_pose,
                const geometry_msgs::PoseStamped &pos,
                const geometry_msgs::PointStamped &vel,
                int ID);
    void PIDPosControl(geometry_msgs::Twist &velocity_ctrl_,\
                       const geometry_msgs::Pose &goal_pose_,\
                       const geometry_msgs::PoseStamped &current_pose_,\
                       const geometry_msgs::PointStamped &current_vel_,\
                       double &yaw);
    //void PIDinit();
    void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
    bool IsArrived(const geometry_msgs::Pose &target,\
                   const geometry_msgs::PoseStamped &pose,\
                   const geometry_msgs::PointStamped &vel);

    void PoseCb_1(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void VelCb_1(const geometry_msgs::PointStamped::ConstPtr &msg);
    void PoseCb_2(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void VelCb_2(const geometry_msgs::PointStamped::ConstPtr &msg);
    void PoseCb_3(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void VelCb_3(const geometry_msgs::PointStamped::ConstPtr &msg);
    void TgtCb_1(const geometry_msgs::Pose::ConstPtr &msg);
    void TgtCb_2(const geometry_msgs::Pose::ConstPtr &msg);
    void TgtCb_3(const geometry_msgs::Pose::ConstPtr &msg);

  private: 
    ros::NodeHandle nh_;
    ros::Subscriber sub_pos_1, sub_pos_2, sub_pos_3;
    ros::Subscriber sub_vel_1, sub_vel_2, sub_vel_3;
    ros::Subscriber sub_tgt_1, sub_tgt_2, sub_tgt_3;
    ros::Publisher pub_cmd_1, pub_cmd_2, pub_cmd_3;
    geometry_msgs::PoseStamped msg_pos_1, msg_pos_2, msg_pos_3;
    geometry_msgs::PointStamped msg_vel_1, msg_vel_2, msg_vel_3;
    geometry_msgs::Twist msg_cmd_1, msg_cmd_2, msg_cmd_3;
    geometry_msgs::Vector3 euler_1, euler_2, euler_3;
    geometry_msgs::Pose target_1, target_2, target_3;
    /*
    Pose --> target
        position:x --> x_target
        position:y --> y_target
        position:z --> z_target
        orientation:x --> vx_target
        orientation:y --> vy_target
        orientation:z --> vz_target
        orientation:w --> yaw_target
    */

    //std::vector<geometry_msgs::Twist> target_list;
    int num_point;
    double Hz;
    int time_hover;
    bool debug;
    double min_xy;
    double max_xy;
    double min_z;
    double max_z;
    double default_z;
    double cmd_xy_max;
    double cmd_z_max;
    double cmd_yaw_max;

    double K_p_xy, K_d_xy, K_p_z, K_d_z, K_p_yaw;
    double eps_xy, eps_z, eps_yaw, eps_v; // yaw:rad  xyz:m

    double del_t; //second: s

};
}
