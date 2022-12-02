#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <memory>
#include <random>
#include "sqlite_orm/sqlite_orm.h"
#include "gps_msgs/SystemState.h"
#include "geodeticconverter.hpp"
#include "mapping_structure.h"
#include "position_handler.h"

static PositionHandler g_posHandler;

void systemStateCallback(const gps_msgs::SystemState::ConstPtr &msg)
{
    g_posHandler.setNewPosition(msg);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "mapping_visualization");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  std::string databaseFile;
  privateNode.param<std::string>("database_file", databaseFile, std::string("mapping_database.sqlite"));
  bool showDetection;
  privateNode.param<bool>("show_detection", showDetection, false);

  ROS_INFO("Opening database under: %s", databaseFile.c_str());
  std::shared_ptr<decltype (openAndPrepareDatabase(""))> ptr_Storage;
  try
  {
      ptr_Storage = std::make_shared<decltype (openAndPrepareDatabase(""))>(openAndPrepareDatabase(databaseFile.c_str()));
  }
  catch (const std::exception &exp)
  {
      ROS_FATAL("Error during opening database: %s", exp.what());
      return 0;
  }

  ros::Subscriber subSystemState = node.subscribe("/SystemState", 1, systemStateCallback);
  ros::Publisher pubMapLines = node.advertise<visualization_msgs::MarkerArray>("/mapping/map_lines", 10);
  ros::Publisher pubDetectionLines = node.advertise<visualization_msgs::MarkerArray>("/mapping/detection_lines", 10);

  std::vector<DataModel::Line> lines = ptr_Storage->get_all<DataModel::Line>();
  std::vector<DataModel::LinePoint> linePoints;
  std::vector<DataModel::LineDetection> lineDetection;

  g_posHandler.setFetchingFunction([&](GeoPoint min, GeoPoint max){
      using namespace sqlite_orm;

      linePoints = ptr_Storage->get_all<DataModel::LinePoint>(where(
                                                  c(&DataModel::LinePoint::lon) > min.lon and c(&DataModel::LinePoint::lon) < max.lon and
                                                  c(&DataModel::LinePoint::lat) > min.lat and c(&DataModel::LinePoint::lat) < max.lat));
      lineDetection = ptr_Storage->get_all<DataModel::LineDetection>(where(
                                                                c(&DataModel::LineDetection::lon) > min.lon and c(&DataModel::LineDetection::lon) < max.lon and
                                                                c(&DataModel::LineDetection::lat) > min.lat and c(&DataModel::LineDetection::lat) < max.lat));

  }, 250, 20);


  ros::Rate loopRate(10);

  std::map<int, visualization_msgs::Marker::_color_type> mapOfColors;
  std::srand(0);
  for(int i = 0; i < 10000; ++i)
  {
      mapOfColors[i].a = 1.0;
      mapOfColors[i].r = std::rand() / (float)RAND_MAX;
      mapOfColors[i].g = std::rand() / (float)RAND_MAX;
      mapOfColors[i].b = std::rand() / (float)RAND_MAX;
  }

  while (ros::ok())
  {
      if (g_posHandler.getConverter() )
      {
          //g_posHandler.interpolatePosition(ros::Time::now());

          visualization_msgs::MarkerArray mapLinesMarkerArray;
          for(const auto &line : lines )
          {
              visualization_msgs::Marker lineMarker;
              lineMarker.header.frame_id = "/mapping";
              lineMarker.header.stamp = ros::Time::now();

              lineMarker.ns = "/mapping/map";
              lineMarker.id = line.id;
              lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
              lineMarker.action = visualization_msgs::Marker::MODIFY;
              lineMarker.pose.orientation.w = 1;
              lineMarker.scale.x = 0.05;
              lineMarker.lifetime = ros::Duration(0.2);
              lineMarker.color = mapOfColors[line.id];

              for(const auto &data : linePoints )
              {
                  if (line.id == *data.lineId)
                  {
                      Point inCarSpace = g_posHandler.getConverter()->geodeticToCarSpace(data);
                      geometry_msgs::Point  visuPoint;
                      visuPoint.x = inCarSpace.x;
                      visuPoint.y = inCarSpace.y;
                      visuPoint.z = inCarSpace.z;
                      lineMarker.points.push_back(visuPoint);
                      //ROS_INFO("Point: %f %f %f | %f %f %f", data.lon, data.lat, data.alt, visuPoint.x, visuPoint.y, visuPoint.z);
                  }
              }

              mapLinesMarkerArray.markers.push_back(lineMarker);
          }

          pubMapLines.publish(mapLinesMarkerArray);


          if (showDetection)
          {
              visualization_msgs::MarkerArray detetcionLinesMarkerArray;
              for(const auto &detetction : lineDetection )
              {
                  visualization_msgs::Marker lineMarker;
                  lineMarker.header.frame_id = "/mapping";
                  lineMarker.header.stamp = ros::Time::now();

                  lineMarker.ns = "/mapping/detection";
                  lineMarker.id = detetction.id;
                  lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
                  lineMarker.action = visualization_msgs::Marker::MODIFY;
                  lineMarker.lifetime = ros::Duration(0.2);
                  lineMarker.pose.orientation.w = 1;
                  lineMarker.scale.x = 0.05;
                  lineMarker.color.a = 1.0;
                  switch (detetction.color) {
                    case 0: //yellow
                      lineMarker.color.r = 1.0;
                      lineMarker.color.g = 1.0;
                      lineMarker.color.b = 0.0;
                      break;
                    case 1: //red
                      lineMarker.color.r = 1.0;
                      lineMarker.color.g = 0.0;
                      lineMarker.color.b = 0.0;
                      break;
                    case 2: //green
                      lineMarker.color.r = 0.0;
                      lineMarker.color.g = 1.0;
                      lineMarker.color.b = 0.0;
                      break;
                    case 3: //blue
                      lineMarker.color.r = 0.0;
                      lineMarker.color.g = 0.0;
                      lineMarker.color.b = 1.0;
                      break;
                    default: //white
                      lineMarker.color.r = 1.0;
                      lineMarker.color.g = 1.0;
                      lineMarker.color.b = 1.0;
                      break;
                  }

                  GeodeticConverter converter(detetction);

                  for(int i = 4 ; i < 40; i+= 4 )
                  {
                      Point artPoint;
                      artPoint.x = i;
                      artPoint.y = detetction.calc(artPoint.x);
                      artPoint.z = 0.0;
                      GeoPoint artGeoPoint = converter.carSpaceToGeodetic(artPoint);
                      //ROS_INFO("Point: %f %f %f", std::fabs(revArtGeoPoint.x - artPoint.x), std::fabs(revArtGeoPoint.y - artPoint.y), std::fabs(revArtGeoPoint.z - artPoint.z));
                      Point inCarSpace = g_posHandler.getConverter()->geodeticToCarSpace(artGeoPoint);
                      geometry_msgs::Point  visuPoint;
                      visuPoint.x = inCarSpace.x;
                      visuPoint.y = inCarSpace.y;
                      visuPoint.z = inCarSpace.z;
                      lineMarker.points.push_back(visuPoint);
                      //ROS_INFO("Point: %f %f %f", visuPoint.x, visuPoint.y, visuPoint.z);
                  }

                  detetcionLinesMarkerArray.markers.push_back(lineMarker);
              }
              pubDetectionLines.publish(detetcionLinesMarkerArray);
          }

      }

      ros::spinOnce();
      loopRate.sleep();
  }


}
