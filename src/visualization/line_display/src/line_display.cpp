/*
 * Copyright (c) 2018, FEV Polska. All rights reserved.
 *
 * ROS node for displaying detected roadsigns in rviz
 * log:
 * 23-01-2018: initial version, glowacki@fev.com
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <cmath>
#include "vision_msgs/ld_line.h"
#include "vision_msgs/ld_data.h"

constexpr char kFrameID[] = "/line_display";
constexpr char kLineDetector[] = "adas_node/ld_data";

using namespace std;

struct Poly { //struct for holding a single line
  double a, b, c, d;
  double red, green, blue;
  float x_range;
};

Poly straight_line, dashed_line; //two types of line
vector<Poly> line_vector;

void SetLineColor(int color, Poly &line)
{
    switch (color) {
      case 0: //yellow
        line.red = 1.0;
        line.green = 1.0;
        line.blue = 0.0;
        break;
      case 1: //red
        line.red = 1.0;
        line.green = 0.0;
        line.blue = 0.0;
        break;
      case 2: //green
        line.red = 0.0;
        line.green = 1.0;
        line.blue = 0.0;
        break;
      case 3: //blue
        line.red = 0.0;
        line.green = 0.0;
        line.blue = 1.0;
        break;
      default: //white
        line.red = 1.0;
        line.green = 1.0;
        line.blue = 1.0;
        break;
    }
}

void line_callback(const vision_msgs::ld_data::ConstPtr &msg)
{
    line_vector.clear();
    for(int i = 0; i < msg->lines.size(); i++)
    {
      straight_line.a = msg->lines.at(i).a;
      straight_line.b = msg->lines.at(i).b;
      straight_line.c = msg->lines.at(i).c;
      straight_line.d = msg->lines.at(i).d;
      SetLineColor(msg->lines.at(i).color, straight_line);
      straight_line.x_range = msg->lines.at(i).length;

      line_vector.push_back(straight_line);
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "line_display");
  ros::NodeHandle n;
  ros::NodeHandle param_handl(n, "roadsigns");

  ros::Subscriber sub = n.subscribe("/adas_node/ld_data", 1000, line_callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/line_display", 10);


  float step_size = 0.1;
  int frame_id = 0;
  ros::Rate r(30);
  int oldMarekerArraySize = 0;
  while (ros::ok())
  {
    frame_id = 0;
    visualization_msgs::MarkerArray markerArray;
    for(int i=0; i < line_vector.size(); i++)
    {
      visualization_msgs::Marker line_strip;
      line_strip.header.frame_id = "/car_cam";
      line_strip.header.stamp = ros::Time::now();
      line_strip.ns = "/car_view";
      line_strip.action = visualization_msgs::Marker::ADD;
      line_strip.pose.orientation.w = 1;

      line_strip.id = frame_id;
      frame_id++;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.05;
      line_strip.color.r = line_vector.at(i).red;//line_vector.at(i).red;
      line_strip.color.g = line_vector.at(i).green;//line_vector.at(i).green;
      line_strip.color.b = line_vector.at(i).blue;//line_vector.at(i).blue;
      line_strip.color.a = 1.0;

      double a = line_vector.at(i).a;
      double b = line_vector.at(i).b;
      double c = line_vector.at(i).c;
      double d = line_vector.at(i).d;
      float x_range = line_vector.at(i).x_range;//line_vector.at(i).x_range / 2;
      float j = 0;//-x_range;

      step_size=x_range/1024.0f;

      while(true)
      {
    	  float y = a*j*j*j + b*j*j + c*j + d;
          	  float z = 0;
              geometry_msgs::Point p;
              p.x = j / 1.0;
              p.y = y / 1.0;
              p.z = z / 1.0;
    	  line_strip.points.push_back(p);

    	  j += step_size;
    	  if ( j >= x_range)  break;
      }
    markerArray.markers.push_back(line_strip);
    printf("line %d strip size %d\n",frame_id,line_strip.points.size());
    }
    for(int i=line_vector.size(); i < 10; i++)
    {
      visualization_msgs::Marker line_strip;
      line_strip.header.frame_id = "/car_cam";
      line_strip.header.stamp = ros::Time::now();
      line_strip.ns = "/car_view";
      line_strip.action = visualization_msgs::Marker::DELETE;
      line_strip.pose.orientation.w = 1;

      line_strip.id = i;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      markerArray.markers.push_back(line_strip);
    }
    marker_pub.publish(markerArray);


    r.sleep();
    ros::spinOnce();
  }
}
