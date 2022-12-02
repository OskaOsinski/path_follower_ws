/*******************************************************************************
 * 
 * utm_converter_node.cpp
 * Created July 25, 2017
 * FEV North America
 * 
 * ROS node wrapper for UTMConverter
 * 
 ******************************************************************************/

#include "UTMConverter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "utm_converter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  gps_waypoint_navigation::UTMConverter node(n, pn);
  
  ros::spin();
}
