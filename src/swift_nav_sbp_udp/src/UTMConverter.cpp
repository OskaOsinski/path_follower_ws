/*******************************************************************************
 * 
 * UTMConverter.cpp
 * Created July 25, 2017
 * FEV North America
 * 
 * What does this do?
 * 
 ******************************************************************************/

#include "UTMConverter.h"

namespace gps_waypoint_navigation
{

void UTMConverter::recvfix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  UTMCoords current_utm(*msg);
  LatLon current_lla(*msg);
  conv_angle_=atan(tan((current_lla.getLon()-current_utm.getCentralMeridian())*M_PI / 180.0)*sin(current_lla.getLat()*M_PI / 180.0));
  relative_position_ = current_utm-ref_utm_;

  geometry_msgs::PoseStamped new_path_point;
  new_path_point.pose.position.x=relative_position_.x();
  new_path_point.pose.position.y=relative_position_.y();
  
  gps_path_.header.frame_id = "world";
  gps_path_.header.stamp = ros::Time::now();
  gps_path_.poses.push_back(new_path_point);
  pub_path_.publish(gps_path_);

  geometry_msgs::PointStamped currentGpsPoint; // final points calculated - HA
  currentGpsPoint.point.x = relative_position_.x();
  currentGpsPoint.point.y = relative_position_.y();
  pub_currentPosition_.publish(currentGpsPoint);
  currentGpsPoint.header.frame_id = "world";
  currentGpsPoint.header.stamp = ros::Time::now();
}


void UTMConverter::recvheading(const gps_msgs::SystemStateConstPtr& msg)
{
  enu_heading_=((((msg->orientation_rad.heading)*(180.0/M_PI))*-1.0)+90)* M_PI / 180.0;
  enu_heading_ +=conv_angle_ ;
}


void UTMConverter::timerCallback50hz(const ros::TimerEvent& e)
{
  std_msgs::Float64 audi_heading;
  visualization_msgs::Marker marker_current_position;

  audi_heading.data = enu_heading_;
  pub_heading_.publish(audi_heading);
  
  marker_current_position.header.frame_id = "world";
  marker_current_position.action = visualization_msgs::Marker::ADD;
  marker_current_position.type = visualization_msgs::Marker::ARROW;
  marker_current_position.pose.position.x = relative_position_.x();
  marker_current_position.pose.position.y = relative_position_.y();
  marker_current_position.pose.orientation = tf::createQuaternionMsgFromYaw(enu_heading_);
			  
  marker_current_position.scale.x = 3;
  marker_current_position.scale.y = 1;
  marker_current_position.scale.z = 0.5;

  marker_current_position.color.a = 1.0;
  marker_current_position.color.r = 1.0;
  marker_current_position.color.g = 1.0;
  marker_current_position.color.b = 0.0;
  pub_marker_current_position_.publish(marker_current_position);
}

UTMConverter::UTMConverter(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
{
  // Initializations:
  const double waypoints_llh_[8][3] = {
    {42.851358,-83.069485,0},//waypint#1
    {42.851383,-83.069007,0},//waypint#2
    {42.852443,-83.068013,0},//waypint#3
    {42.852021,-83.066888,0},//waypint#4
    {42.851525,-83.067044,0},//waypint#5
    {42.851344,-83.066344,0},//waypint#6
    {42.850836,-83.066440,0},//waypint#7
    {42.849644,-83.066060,0},//waypint#8
  };
  
  // Parameters
  priv_nh.getParam("ref_lat_param", ref_lat_);
  priv_nh.getParam("ref_long_param", ref_lon_);
  LatLon llh(ref_lat_,ref_lon_,0);
  ref_utm_ = UTMCoords(llh);
  double x,y;
  int zone,hemi;
  for(int i=0;i<8;i++)
  {
    LLtoUTM(waypoints_llh_[i][0], waypoints_llh_[i][1],x, y, zone, hemi);
    waypoint_utm_[i].setX(x);
    waypoint_utm_[i].setY(y);
    waypoint_utm_[i].setZone(zone);
    waypoint_utm_[i].setHemi(hemi);
    waypoint_relative_position_[i] =waypoint_utm_[i]-ref_utm_;
  }
  
  // Subscribers
  sub_fix_ = node.subscribe("/NavSatFix",1, &UTMConverter::recvfix, this);
  sub_heading_ = node.subscribe("/SystemState",1, &UTMConverter::recvheading, this);
  
  // Publishers
  pub_path_ = node.advertise<nav_msgs::Path>("gps_path",1);
  pub_currentPosition_ = node.advertise<geometry_msgs::PointStamped>("audibot/current_position",1);
  pub_heading_ = node.advertise<std_msgs::Float64>("audibot_enu_heading", 1);
  pub_marker_current_position_ = node.advertise<visualization_msgs::Marker>("/marker_current_position", 1);
  // Timer
  timer50hz_ = node.createTimer(ros::Duration(0.02), &UTMConverter::timerCallback50hz, this);
}

} // namespace gps_waypoint_navigation
