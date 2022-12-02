/*******************************************************************************
 * 
 * UTMConverter.h
 * Created July 25, 2017
 * FEV North America
 * 
 * Definitions for UTMConverter
 * 
 ******************************************************************************/

// Include guard
#ifndef UTMCONVERTER_H
#define UTMCONVERTER_H

// Includes
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include "gps_conv.h"
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PoseArray.h>
#include <gps_msgs/SystemState.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <vector>
#include <iostream>

namespace gps_waypoint_navigation
{

class UTMConverter
{
public:
  // Constructor
  UTMConverter(ros::NodeHandle &node, ros::NodeHandle &priv_nh);

private:
  // Functions
  void recvfix(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvheading(const gps_msgs::SystemStateConstPtr& msg);
  void timerCallback50hz(const ros::TimerEvent& e);
  
  // Publishers
  ros::Publisher pub_path_;
  ros::Publisher pub_currentPosition_;
  ros::Publisher pub_heading_;
  ros::Publisher pub_marker_current_position_;
  // Subscribers
  ros::Subscriber sub_fix_;
  ros::Subscriber sub_heading_;
  
  // Timers
  ros::Timer timer50hz_;
  
  // Globals
  double ref_lat_; 
  double ref_lon_; 
  double conv_angle_;
  double enu_heading_;
  nav_msgs::Path gps_path_;
  tf::Vector3 relative_position_;
  UTMCoords ref_utm_;
  UTMCoords waypoint_utm_[8];
  tf::Vector3 waypoint_relative_position_[8];
  
}; // class UTMConverter
} // namespace gps_waypoint_navigation

#endif // UTMCONVERTER_H
