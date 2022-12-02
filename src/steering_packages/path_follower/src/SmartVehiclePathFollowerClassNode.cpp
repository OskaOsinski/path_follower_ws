#include <ros/ros.h>
#include <SmartVehiclePathFollowerClass.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_node");
    ros::NodeHandle nh;
    ROS_INFO("steering_node run");
    SmartVehicleSteeringClass node(nh);
    ros::spin();

    return 0;
}  
