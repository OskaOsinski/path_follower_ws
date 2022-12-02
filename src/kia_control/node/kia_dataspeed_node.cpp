#include <ros/ros.h>

#include "kia_dataspeed_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kia_dataspeed");

  ros::NodeHandlePtr public_nh( new ros::NodeHandle());
  ros::NodeHandlePtr private_nh( new ros::NodeHandle("~"));

  KiaDataspeedController controller(public_nh, private_nh);

  ros::spin();

  ros::waitForShutdown();
}
