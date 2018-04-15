#include "flight3ardrone/flight3ardrone.h"

namespace Mul_Drone_Ctrl
{
drone_ctrl::drone_ctrl(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh)
{
    pnh.param("time_hover", time_hover, 20000);
    pnh.param("debug", debug, true);
    pnh.param("Hz", Hz, 20.0);
    del_t = 1.0 / Hz;

    pnh.param("K_p_xy", K_p_xy, 0.2);
    pnh.param("K_p_z", K_p_z, 0.2);
    pnh.param("K_p_yaw", K_p_yaw, 0.2);
    pnh.param("K_d_xy", K_d_xy, 0.4);
    pnh.param("K_d_z", K_d_z, 0.2);

    pnh.param("min_xy", min_xy, 0.5);
    pnh.param("max_xy", max_xy, 3.5);
    pnh.param("min_z", min_z, 0.8);
    pnh.param("max_z", max_z, 2.5);
    pnh.param("default_z", default_z, 1.2);

    pnh.param("cmd_xy_max", cmd_xy_max, 0.5);
    pnh.param("cmd_z_max", cmd_z_max, 0.5);
    pnh.param("cmd_yaw_max", cmd_yaw_max, 0.5);

    pnh.param("eps_xy", eps_xy, 0.3);
    pnh.param("eps_z", eps_z, 0.2);
    pnh.param("eps_yaw", eps_yaw, 0.5);

    pub_cmd_1 = nh_.advertise<geometry_msgs::Twist>("/ardrone1/cmd_vel", 1);
    pub_cmd_2 = nh_.advertise<geometry_msgs::Twist>("/ardrone2/cmd_vel", 1);
    pub_cmd_3 = nh_.advertise<geometry_msgs::Twist>("/ardrone3/cmd_vel", 1);

    sub_pos_1 = nh_.subscribe("/ardrone1/pos_uav", 2, &drone_ctrl::PoseCb_1, this);
    sub_vel_1 = nh_.subscribe("/ardrone1/vel_uav", 2, &drone_ctrl::VelCb_1, this);
    sub_pos_2 = nh_.subscribe("/ardrone2/pos_uav", 2, &drone_ctrl::PoseCb_2, this);
    sub_vel_2 = nh_.subscribe("/ardrone2/vel_uav", 2, &drone_ctrl::VelCb_2, this);
    sub_pos_3 = nh_.subscribe("/ardrone3/pos_uav", 2, &drone_ctrl::PoseCb_3, this);
    sub_vel_3 = nh_.subscribe("/ardrone3/vel_uav", 2, &drone_ctrl::VelCb_3, this);
    sub_tgt_1 = nh_.subscribe("/ardrone1/target", 2, &drone_ctrl::TgtCb_1, this);
    sub_tgt_2 = nh_.subscribe("/ardrone2/target", 2, &drone_ctrl::TgtCb_2, this);
    sub_tgt_3 = nh_.subscribe("/ardrone3/target", 2, &drone_ctrl::TgtCb_3, this);
    ros::Rate loopRate(Hz);
    while (ros::ok())
    {
        //usleep(10000); // 10000ms can not receive correct data
        if (drone_ctrl::IsArrived(target_1, msg_pos_1, msg_vel_1))
            drone_ctrl::GenCmd(msg_cmd_1, target_1, msg_pos_1, msg_vel_1, 1);
        if (drone_ctrl::IsArrived(target_2, msg_pos_2, msg_vel_2))
            drone_ctrl::GenCmd(msg_cmd_2, target_2, msg_pos_2, msg_vel_2, 2);
        if (drone_ctrl::IsArrived(target_3, msg_pos_3, msg_vel_3))
            drone_ctrl::GenCmd(msg_cmd_3, target_3, msg_pos_3, msg_vel_3, 3);
        loopRate.sleep();
        ros::spinOnce();
    }
}

void drone_ctrl::GenCmd(geometry_msgs::Twist &cmd,
                      const geometry_msgs::Pose &goal_pose,
                      const geometry_msgs::Pose &pos,
                      const geometry_msgs::Point &vel,
                      int ID)
{
    cout << "generating the vel cmd of drone:"<< ID << endl;
  
    geometry_msgs::Vector3 euler_tmp;
    drone_ctrl::Quat2Euler(pos, euler_tmp);

    drone_ctrl::PIDPosControl(cmd, goal_pose, pos, vel, euler_tmp.z);
}

void drone_ctrl::PIDPosControl(geometry_msgs::Twist &velocity_ctrl_,
                               const geometry_msgs::Pose &goal_pose_,
                               const geometry_msgs::PoseStamped &current_pose_,
                               const geometry_msgs::PointStamped &current_vel_,
                               double &yaw)
{

    double vel_x = current_vel_.point.x;
    double vel_y = current_vel_.point.y;
    double vel_z = current_vel_.point.z;
    double error_x, error_y, error_z, error_yaw, error_vx, error_vy, error_vz;
    error_x = goal_pose_.position.x - current_pose_.pose.position.x;
    error_y = goal_pose_.position.y - current_pose_.pose.position.y;
    error_z = goal_pose_.position.z - current_pose_.pose.position.z;
    error_vx = goal_pose_.orientation.x - current_vel_.point.x;
    error_vy = goal_pose_.orientation.y - current_vel_.point.y;
    error_vz = goal_pose_.orientation.z - current_vel_.point.z;
    error_yaw = goal_pose_.orientation.w - yaw;

    cmd_vel_x = K_p_xy * error_x + K_d_xy * error_vx;
    cmd_vel_y = K_p_xy * error_y + K_d_xy * error_vy;
    cmd_vel_z = K_p_z * error_z + K_d_z * error_vz;
    cmd_vel_yaw = K_p_yaw * error_yaw;

    // TODO:need calculated ????
    velocity_ctrl_.linear.x = cos(error_yaw) * cmd_vel_x + sin(error_yaw) * cmd_vel_y;
    velocity_ctrl_.linear.y = sin(error_yaw) * cmd_vel_x + cos(error_yaw) * cmd_vel_y;

    velocity_ctrl_.linear.x = CLAMP(velocity_ctrl_.linear.x, -cmd_xy_max, cmd_xy_max);
    velocity_ctrl_.linear.y = CLAMP(velocity_ctrl_.linear.y, -cmd_xy_max, cmd_xy_max);
    velocity_ctrl_.linear.z = CLAMP(cmd_vel_z, -cmd_z_max, cmd_z_max);
    velocity_ctrl_.angular.z = CLAMP(cmd_vel_yaw, -cmd_yaw_max, cmd_yaw_max);
    /*
        linear.x (+)  fly forward
                (-)  fly backward
        linear.y (+)  fly left
                (-)  fly right
        linear.z (+)  fly up
                (-)  fly down
        angular.z(+)  rotate counter clockwise    
                (-)  rotate clockwise 
        roll_degree = linear.y * max_tilt_angle 
        pitch_degree = linear.x * max_tilt_angle 
        ver_vel_m_per_s = linear.z * max_vert_speed 
        rot_vel_deg_per_s = angular.z * max_rot_speed       
        */

    if (debug)
    {
        cout << "control velocity =:" << endl;
        cout << "V_x =:" << velocity_ctrl_.linear.x << "  ";
        cout << "V_y =:" << velocity_ctrl_.linear.y << "  ";
        cout << "V_z =:" << velocity_ctrl_.linear.z << "  ";
        cout << "V_yaw =:" << velocity_ctrl_.angular.z << endl;
    }
    velocity_ctrl_.linear.y = -velocity_ctrl_.linear.y;
    velocity_ctrl_.linear.z = -velocity_ctrl_.linear.z;
    velocity_ctrl_.angular.x = 0.0;
    velocity_ctrl_.angular.y = 0.0;
    velocity_ctrl_.angular.z = -velocity_ctrl_.angular.z;
}
//TODO:
void bebop_pos_ctrl::BebopPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_sub = *msg;
}

void bebop_pos_ctrl::BebopVelCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    vel_sub = *msg;
}

void bebop_pos_ctrl::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
{
    double q0 = quat.w;
    double q1 = quat.x;
    double q2 = quat.y;
    double q3 = quat.z;
    double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
    double t1 = +2.0 * (q1 * q2 + q0 * q3);
    double t2 = -2.0 * (q1 * q3 - q0 * q2);
    double t3 = +2.0 * (q2 * q3 + q0 * q1);
    double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;
    //t2 = t2 > 1.0 ? 1.0 : t2;
    //t2 = t2 < -1.0 ? -1.0 : t2;
    // cout<<"pitch =  :"<< asin(t2)<<endl;
    // cout<<"roll  =  :"<< atan2(t3, t4)<<endl;
    // cout<<"yaw   =  :"<< atan2(t1, t0)<<endl;
    euler.x = asin(t2);
    euler.y = -atan2(t3, t4);
    euler.z = atan2(t1, t0);
}

void bebop_pos_ctrl::Euler2Quat(geometry_msgs::Vector3 &euler, geometry_msgs::Quaternion &quat)
{
    double fi = euler.x / 2;
    double theta = euler.y / 2;
    double psi = euler.z / 2;
    quat.w = cos(fi) * cos(theta) * cos(psi) + sin(fi) * sin(theta) * sin(psi);
    quat.x = sin(fi) * cos(theta) * cos(psi) - cos(fi) * sin(theta) * sin(psi);
    quat.y = cos(fi) * sin(theta) * cos(psi) + sin(fi) * cos(theta) * sin(psi);
    quat.z = cos(fi) * cos(theta) * sin(psi) - sin(fi) * sin(theta) * cos(psi);
}
void bebop_pos_ctrl::Limitator(double &vx, double &vy, double &vz, double &yawrate)
{
    if (vx < (-MaxV_xy))
        vx = -MaxV_xy;
    if (vx > MaxV_xy)
        vx = MaxV_xy;

    if (vy < (-MaxV_xy))
        vy = -MaxV_xy;
    if (vy > MaxV_xy)
        vy = MaxV_xy;

    if (vz < (-MaxV_z))
        vz = -MaxV_z;
    if (vz > MaxV_z)
        vz = MaxV_z;

    if (yawrate < (-MaxV_yaw))
        yawrate = -MaxV_yaw;
    if (yawrate > MaxV_yaw)
        yawrate = MaxV_yaw;
}
}