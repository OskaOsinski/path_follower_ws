#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <memory>
#include <limits>
#include "sqlite_orm/sqlite_orm.h"
#include "vision_msgs/od_data.h"
#include "vision_msgs/ld_data.h"
#include "gps_msgs/SystemState.h"
#include "geodeticconverter.hpp"
#include "mapping_structure.h"
#include "position_handler.h"

namespace orm = sqlite_orm;

static std::shared_ptr<decltype (openAndPrepareDatabase(""))> ptr_Storage;
static PositionHandler g_posHandler;

static std::vector<DataModel::Line> g_lines;
static std::vector<DataModel::LinePoint> g_linePoints;
static std::vector<DataModel::LinePoint> g_visiblePoints;

constexpr double X_VALUE_OF_MOST_ACCURATE_POINT = 10.0;
constexpr double X_VALUE_OF_NEW_LINE_FIRST_POINT = 6.0;
constexpr double X_VALUE_OF_NEW_LINE_SECOND_POINT = 8.0;
constexpr double MAX_VERTICAL_DISTANCE_BETWEEN_LINE_POINTS = 2.0;
constexpr double MAX_DISTANCE_BETWEEN_LINE_POINTS = 8.0;
constexpr double MIN_DISTANCE_BETWEEN_LINE_POINTS = 4.0;
constexpr double MAX_ANGLE_BETWEEN_NEW_POINT = 75.0;

double calculateLineValue(const vision_msgs::ld_line &line, double x)
{
    return line.a*x*x*x + line.b*x*x + line.c*x + line.d;
}


void objectCallback(const vision_msgs::od_data::ConstPtr &msg)
{

}

struct PointHelper
{
    DataModel::LinePoint data;
    Point inCarSpace;
    int color;
};

struct LinePointsHelper
{
    PointHelper last;
    double distanceToLast;
    PointHelper beforeLast;
    double distanceToBeforeLast;
};

void lineCallback(const vision_msgs::ld_data::ConstPtr &msg)
{
    if (! g_posHandler.getConverter() )
    {
        ROS_WARN("Invalid geodetic converter.");
        return;
    }

    //g_posHandler.interpolatePosition(ros::Time::now());

    std::vector<DataModel::Line> addedLines;
    int newLineTempIndex = 0;
    std::map<int, std::vector<DataModel::LinePoint>> addedPointsToNewLine;
    std::vector<DataModel::LinePoint> addedPointsToOldLine;
    std::vector<DataModel::LineDetection> allLineDetection;
    std::vector<PointHelper> vectorVisiblePoints;
    for (auto & point : g_visiblePoints)
    {
        PointHelper helper;
        helper.data = point;
        helper.inCarSpace = g_posHandler.getConverter()->geodeticToCarSpace(point);
        helper.color = std::find_if(g_lines.begin(), g_lines.end(), [&](const DataModel::Line &line){ return line.id == *point.lineId;})->color;
        vectorVisiblePoints.push_back(helper);
    }

    CarPosition actualPosition = g_posHandler.getConverter()->getHome();

    for(auto & line : msg->lines)
    {
        if (line.valid)
        {
            DataModel::LineDetection detectedLine;
            detectedLine.lon = actualPosition.point.lon;
            detectedLine.lat = actualPosition.point.lat;
            detectedLine.alt = actualPosition.point.h;
            detectedLine.az = actualPosition.azimuthReal;
            detectedLine.type = line.type;
            detectedLine.color = line.color;
            detectedLine.a = line.a;
            detectedLine.b = line.b;
            detectedLine.c = line.c;
            detectedLine.d = line.d;
            allLineDetection.push_back(detectedLine);

            Point linePoint;
            linePoint.x = X_VALUE_OF_MOST_ACCURATE_POINT;
            linePoint.y = calculateLineValue(line, linePoint.x);
            linePoint.z = 0.0;

            // Find in visible line points nearest line
            std::shared_ptr<int> bestMatchLineId;
            double nearestPoint = std::numeric_limits<double>::max();
            std::map<int,LinePointsHelper> nearestPointOfEveryLine;

            for (auto & pointHelper : vectorVisiblePoints)
            {
                if (line.color == pointHelper.color)
                {
                    double deltaX = pointHelper.inCarSpace.x - linePoint.x;
                    double deltaY = pointHelper.inCarSpace.y - linePoint.y;
                    double deltaZ = pointHelper.inCarSpace.z - linePoint.z;
                    double distance = std::sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

                    // NOTE: Ignore line point which are to for from us
                    if (std::abs(deltaZ) > MAX_VERTICAL_DISTANCE_BETWEEN_LINE_POINTS ||
                        distance > (MAX_DISTANCE_BETWEEN_LINE_POINTS*2.0) )
                        //distance < MIN_DISTANCE_BETWEEN_LINE_POINTS)
                    {
                        continue;
                    }

                    if (nearestPointOfEveryLine.count(*pointHelper.data.lineId))
                    {
                        if (distance < nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToLast)
                        {
                            nearestPointOfEveryLine[*pointHelper.data.lineId].beforeLast = nearestPointOfEveryLine[*pointHelper.data.lineId].last;
                            nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToBeforeLast = nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToLast;
                            nearestPointOfEveryLine[*pointHelper.data.lineId].last = pointHelper;
                            nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToLast = distance;
                        }
                    }
                    else
                    {
                        nearestPointOfEveryLine[*pointHelper.data.lineId].last = pointHelper;
                        nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToLast = distance;
                        nearestPointOfEveryLine[*pointHelper.data.lineId].distanceToBeforeLast = -1;
                    }
                }
            }

            for(const auto &pointOfLine : nearestPointOfEveryLine)
            {
                // NOTE: Ingore lines with less than 2 points, beacuse we can't calculate heading angle
                if (pointOfLine.second.distanceToBeforeLast < 0)
                {
                    continue;
                }
                std::vector<Point> points{pointOfLine.second.beforeLast.inCarSpace, pointOfLine.second.last.inCarSpace, linePoint};
                std::sort(points.begin(), points.end(), [](const Point &a, const Point &b) {return a.x < b.x;});

                double angleFirstToSecond = std::atan2(points[1].y - points[0].y, points[1].x - points[0].x);
                double angleSecondToThrid = std::atan2(points[2].y - points[1].y, points[2].x - points[1].x);

                //ROS_INFO("Angle %f %f |", angleBeforeToLast, anglelastToNew);
                ROS_INFO("atan2 %d = %f | %f %f | %f %f | %f %f |", pointOfLine.first ,GeodeticConverter::RAD_TO_DEG(std::abs(angleFirstToSecond - angleSecondToThrid)), GeodeticConverter::RAD_TO_DEG(angleFirstToSecond), GeodeticConverter::RAD_TO_DEG(angleSecondToThrid),
                         points[1].y - points[0].y, points[1].x - points[0].x,
                         points[2].y - points[1].y, points[2].x - points[1].x);
                if (nearestPoint > pointOfLine.second.distanceToLast)
                {
                    if (pointOfLine.second.distanceToLast > MIN_DISTANCE_BETWEEN_LINE_POINTS)
                    {
                        if (std::abs(angleFirstToSecond - angleSecondToThrid) < GeodeticConverter::DEG_TO_RAD(2.0*MAX_ANGLE_BETWEEN_NEW_POINT))
                        {
                            nearestPoint = pointOfLine.second.distanceToLast;
                            bestMatchLineId = std::make_shared<int>(pointOfLine.first);
                        }
                    }
                    else
                    {
                        nearestPoint = pointOfLine.second.distanceToLast;
                        bestMatchLineId = std::make_shared<int>(pointOfLine.first);
                    }
                }

            }
            // NOTE: We don't find any lines for this point so we add new line
            if (!bestMatchLineId)
            {
                DataModel::Line newLine{newLineTempIndex,1,line.color};
                bestMatchLineId = std::make_shared<int>(newLineTempIndex);
                newLine.id = *bestMatchLineId;
                addedLines.push_back(newLine);

                Point linePoint1;
                linePoint1.x = X_VALUE_OF_NEW_LINE_FIRST_POINT;
                linePoint1.y = calculateLineValue(line, linePoint1.x);
                linePoint1.z = 0.0;
                GeoPoint geo1 = g_posHandler.getConverter()->carSpaceToGeodetic(linePoint1);
                DataModel::LinePoint newPoint1 {0,bestMatchLineId,geo1.lon,geo1.lat, geo1.h};
                //newPoint1.id = ptr_Storage->insert(newPoint1);

                Point linePoint2;
                linePoint2.x = X_VALUE_OF_NEW_LINE_SECOND_POINT;
                linePoint2.y = calculateLineValue(line, linePoint2.x);
                linePoint2.z = 0.0;
                GeoPoint geo2 = g_posHandler.getConverter()->carSpaceToGeodetic(linePoint2);
                DataModel::LinePoint newPoint2 {0,bestMatchLineId,geo2.lon,geo2.lat, geo2.h};
                //newPoint2.id = ptr_Storage->insert(newPoint2);

                GeoPoint geo = g_posHandler.getConverter()->carSpaceToGeodetic(linePoint);
                DataModel::LinePoint newPoint3 {0,bestMatchLineId,geo.lon,geo.lat, geo.h};

                addedPointsToNewLine[newLineTempIndex].push_back(newPoint1);
                addedPointsToNewLine[newLineTempIndex].push_back(newPoint2);
                addedPointsToNewLine[newLineTempIndex].push_back(newPoint3);
                ROS_INFO("Created new line temp id: %d", *bestMatchLineId);
                //ROS_INFO("Added new point %f %f | %f %f ", linePoint1.x, linePoint1.y, linePoint2.x, linePoint2.y);
                //ROS_INFO("Added new point %f %f | %f %f ", geo1.lon, geo1.lat, geo2.lon, geo2.lat);

                newLineTempIndex++;
            }
            else
            {
                GeoPoint geo = g_posHandler.getConverter()->carSpaceToGeodetic(linePoint);
                DataModel::LinePoint newPoint {0,bestMatchLineId,geo.lon,geo.lat, geo.h};
                addedPointsToOldLine.push_back(newPoint);
                ROS_INFO("Added to line: %d", *bestMatchLineId);
            }



//                ROS_INFO("Added to line: %d | %f %f | %f %f |", *bestMatchLineId,
//                         nearestPointOfEveryLine[*bestMatchLineId].last.inCarSpace.x,
//                        nearestPointOfEveryLine[*bestMatchLineId].last.inCarSpace.y,
//                        nearestPointOfEveryLine[*bestMatchLineId].beforeLast.inCarSpace.x,
//                        nearestPointOfEveryLine[*bestMatchLineId].beforeLast.inCarSpace.y);

        }
    }

    ptr_Storage->transaction([&]() mutable
    {
        for(const auto &det : allLineDetection)
        {
            ptr_Storage->insert<DataModel::LineDetection>(det);
        }

        for(auto &line : addedLines)
        {
            int tempLineId = line.id;
            line.id = ptr_Storage->insert<DataModel::Line>(line);
            g_lines.push_back(line);

            for(auto &point : addedPointsToNewLine[tempLineId])
            {
                point.lineId = std::make_shared<int>(line.id);
                point.id = ptr_Storage->insert<DataModel::LinePoint>(point);
                g_visiblePoints.push_back(point);
            }
        }

        for(auto &point : addedPointsToOldLine)
        {
            ptr_Storage->insert<DataModel::LinePoint>(point);
            g_visiblePoints.push_back(point);
        }

        return true;
    });

}


void systemStateCallback(const gps_msgs::SystemState::ConstPtr &msg)
{
    g_posHandler.setNewPosition(msg);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "mapping_database");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  std::string databaseFile;
  privateNode.param<std::string>("database_file", databaseFile, std::string("mapping_database.sqlite"));

  ROS_INFO("Opening database under: %s", databaseFile.c_str());
  try
  {
      ptr_Storage = std::make_shared<decltype (openAndPrepareDatabase(""))>(openAndPrepareDatabase(databaseFile.c_str()));
  }
  catch (const std::exception &exp)
  {
      ROS_FATAL("Error during opening database: %s", exp.what());
      return 0;
  }

  ros::Subscriber subOdData = node.subscribe("/adas_node/od_data", 1000, objectCallback);
  ros::Subscriber subLdData = node.subscribe("/adas_node/ld_data", 1000, lineCallback);
  ros::Subscriber subSystemState = node.subscribe("/SystemState", 1000, systemStateCallback);

  g_posHandler.setFetchingFunction([&](GeoPoint min, GeoPoint max){
      using namespace sqlite_orm;
      g_visiblePoints = ptr_Storage->get_all<DataModel::LinePoint>(where(
                                                  c(&DataModel::LinePoint::lon) > min.lon and c(&DataModel::LinePoint::lon) < max.lon and
                                                  c(&DataModel::LinePoint::lat) > min.lat and c(&DataModel::LinePoint::lat) < max.lat));
      g_lines = ptr_Storage->get_all<DataModel::Line>();
  }, 140, 20);

  ptr_Storage->remove_all<DataModel::LinePoint>();
  ptr_Storage->remove_all<DataModel::Line>();

  ros::spin();
}
