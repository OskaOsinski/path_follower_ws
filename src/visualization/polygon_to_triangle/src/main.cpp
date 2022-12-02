#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <kml_parser_publisher/kmlPolygonArray.h>
#include <string>

#include "poly2tri.h"

std::map<std::string, ros::Publisher> markerPub;
std::map<std::string, std_msgs::ColorRGBA> markerOfColor;
int idForFilledPolygons = 12500;
int idForFilledPcl = 12600;

void kmlPolygonCallback(const kml_parser_publisher::kmlPolygonArray::ConstPtr msg, const std::string &topic)
{
    int idIncr = 0;
    for (const auto &kmlPoly : msg->kmlPolygons)
    {
        std::vector<p2t::Point*> inputPoints;
        inputPoints.reserve(kmlPoly.coord.size());
        for(int i = 0 ; i < kmlPoly.coord.size(); i++)
        {
            bool foundDup = false;
            for(int j = 0 ; j < inputPoints.size(); j++)
            {
                if (std::abs(inputPoints[j]->x - kmlPoly.coord[i].x) < 0.5/*std::numeric_limits<double>::epsilon()*/
                   && std::abs(inputPoints[j]->y - kmlPoly.coord[i].y) < 0.5)
                   {
                       foundDup = true;
                       break;  
                   }
            }
            if (! foundDup)
            {
                inputPoints.push_back(new p2t::Point(kmlPoly.coord[i].x, kmlPoly.coord[i].y));
            }
        }
       

        p2t::CDT trian (inputPoints);
        trian.Triangulate();

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.header = kmlPoly.header;
        marker.header.frame_id = "map";
        marker.id = idForFilledPolygons + idIncr++; // Some random id
        marker.ns = "";
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.pose.orientation.w = 1.0;
        if(kmlPoly.description == std::string("availableRoadArea")) 
        {
            marker.color = markerOfColor["availableRoadArea"];
        }
        else if(kmlPoly.description == std::string("availableOther")) 
        {
            marker.color = markerOfColor["availableOther"];
        }
        else if(kmlPoly.description == std::string("building"))
        {
            marker.color = markerOfColor["building"];
        }
        else if (kmlPoly.description == std::string("grass"))
        {
            marker.color = markerOfColor["grass"];
        }
        else if(kmlPoly.description == std::string("unavailableRoadArea"))
        {
            marker.color = markerOfColor["unavailableRoadArea"];
        }
        else
        {
            marker.color.r = 205; 
            marker.color.g = 50;
            marker.color.b = 50;
            marker.color.a = 200;
        }

        marker.points.reserve(kmlPoly.coord.size());

        auto triangles = trian.GetTriangles();
        for(int i = 0 ; i < triangles.size(); ++i)
        {
            // ROS_INFO(" \t%d ", i);
            for(int j = 0; j < 3 ; j++)
            {            
                // ROS_INFO("\t\t %f %f", j, triangles[i]->GetPoint(j)->x, triangles[i]->GetPoint(j)->y);
                geometry_msgs::Point point;
                point.x = triangles[i]->GetPoint(j)->x;
                point.y = triangles[i]->GetPoint(j)->y;
                point.z = 0.0;
                marker.points.push_back(point);
            }
        }
        markerPub[topic].publish(marker);

        // clearing data
        for(int i = 0 ; i < inputPoints.size(); i++)
        {
            delete inputPoints[i];
        }
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr msg, const std::string &topic)
{
    // As first step we downsample contour to speedup processing
    constexpr int downSampleRatio = 16;
    std::vector<geometry_msgs::Point32> filteredPoints;
    filteredPoints.reserve(msg->points.size() / downSampleRatio + 1);
    for(int i = 0 ; i < msg->points.size(); i+=downSampleRatio)
    {
        filteredPoints.push_back(msg->points[i]);
    }

    // Here we are reversing points on the list because freespace
    // send us in clockwise direction and we need to have in counter clockwise
    std::reverse(filteredPoints.begin(), filteredPoints.end());

    std::vector<p2t::Point*> inputPoints;
    inputPoints.reserve(filteredPoints.size());
    for(int i = 0 ; i < filteredPoints.size(); i++)
    {
        //ROS_INFO("Points sorted: %d | %f %f", i, filteredPoints[i].x, filteredPoints[i].y);
        inputPoints.push_back(new p2t::Point(filteredPoints[i].x, filteredPoints[i].y));
    }

    p2t::CDT trian (inputPoints);

    trian.Triangulate();

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.header = msg->header; // We want to have the same timestamp and fixed frame as original data
    marker.id = idForFilledPcl; // Some random id
    marker.ns = "";
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.color = markerOfColor["pcl"];

    marker.points.reserve(filteredPoints.size());

    auto triangles = trian.GetTriangles();
    for(int i = 0 ; i < triangles.size(); ++i)
    {
        // ROS_INFO(" \t%d ", i);
        for(int j = 0; j < 3 ; j++)
        {            
            // ROS_INFO("\t\t %f %f", j, triangles[i]->GetPoint(j)->x, triangles[i]->GetPoint(j)->y);
            geometry_msgs::Point point;
            point.x = triangles[i]->GetPoint(j)->x;
            point.y = triangles[i]->GetPoint(j)->y;
            point.z = 0.0;
            marker.points.push_back(point);
        }
    }
    markerPub[topic].publish(marker);

    // clearing data
    for(int i = 0 ; i < inputPoints.size(); i++)
    {
        delete inputPoints[i];
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "polygon_to_triangles");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle prvNodeHandle("~");

  XmlRpc::XmlRpcValue colorValue;
  XmlRpc::XmlRpcValue colorDefValue;
  prvNodeHandle.param("fill_color", colorValue, colorDefValue);
  if (colorValue.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct)
  {
      for(auto &elem : colorValue)
      {
          //ROS_INFO("%s %d %d", elem.first.c_str(), elem.second.getType(), elem.second.size());
          std_msgs::ColorRGBA color;
          if (elem.second.getType() == XmlRpc::XmlRpcValue::Type::TypeArray 
            && elem.second.size() == 4)
          {
              color.r = (int)(elem.second[0]) / 255.0;
              color.g = (int)(elem.second[1]) / 255.0;
              color.b = (int)(elem.second[2]) / 255.0;
              color.a = (int)(elem.second[3]) / 255.0;
              
          }
          else
          {
              throw std::logic_error("Color must contains list of 4 number [r, g, b, a]");
          }
          
          markerOfColor[elem.first] = color;
      }
  }
  else
{
    throw std::logic_error("Color map must be map of list colorName : [ r, g, b, a]");
}

  std::map<std::string, ros::Subscriber> polySubMMap;
  XmlRpc::XmlRpcValue topicValue;
  XmlRpc::XmlRpcValue defValue;
  prvNodeHandle.param("pcl_topics", topicValue, defValue);
  for(int i = 0; i < topicValue.size(); ++i)
  {
    std::string topicName = topicValue[i];
    ROS_INFO("Making pcl filled polygon for topic: %s", topicName.c_str());
    polySubMMap[topicName] = nodeHandle.subscribe<sensor_msgs::PointCloud>(topicName, 1, boost::bind(pointCloudCallback, _1, topicName));
    markerPub[topicName] = nodeHandle.advertise<visualization_msgs::Marker>(topicName + "_fill", 10);
  }
  
  XmlRpc::XmlRpcValue kmlTopicValue;
  XmlRpc::XmlRpcValue kmlDefValue;
  prvNodeHandle.param("kml_topics", kmlTopicValue, kmlDefValue);
  for(int i = 0; i < kmlTopicValue.size(); ++i)
  {
    std::string topicName = kmlTopicValue[i];
    ROS_INFO("Making kml filled polygon for topic: %s", topicName.c_str());
    polySubMMap[topicName] = nodeHandle.subscribe<kml_parser_publisher::kmlPolygonArray>(topicName, 1, boost::bind(kmlPolygonCallback, _1, topicName));
    markerPub[topicName] = nodeHandle.advertise<visualization_msgs::Marker>(topicName + "_fill", 10);
  }
  
  ros::spin();
}
