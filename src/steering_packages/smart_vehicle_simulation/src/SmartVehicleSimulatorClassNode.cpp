#include <ros/ros.h>
#include <SmartVehicleSimulatorClass.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_node");
    ros::NodeHandle nh;
    ROS_INFO("sim run");
    SmartVehicleSimulatorClass node(nh);
    ros::spin();

    return 0;
}  
