#include "flight3ardrone/flight3ardrone.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mission_drone_ctrl_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Mul_Drone_Ctrl::drone_ctrl Ardrone_Mission(nh, pnh);
    return 0;
}