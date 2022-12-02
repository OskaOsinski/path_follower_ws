#include <ros/ros.h>
#include <string>
#include <memory>
#include <limits>
#include <eigen3/Eigen/QR>
#include "sqlite_orm/sqlite_orm.h"
#include "geodeticconverter.hpp"
#include "mapping_structure.h"
#include "position_handler.h"


using namespace sqlite_orm;

static std::shared_ptr<decltype (openAndPrepareDatabase(""))> ptr_Storage;

void RemoveLinesWith3PointsAndLess()
{
    ROS_INFO("Removing lines with 3 points and less.");
    auto allLines = ptr_Storage->get_all<DataModel::Line>();
    std::vector<int> linesToRemove;
    for (const auto &line : allLines)
    {
        using namespace sqlite_orm;
        int numberOfPoints = ptr_Storage->count(&DataModel::LinePoint::id, where(c(&DataModel::LinePoint::lineId) == line.id));
        //ROS_INFO("Line with id: %d points count: %d", line.id, numberOfPoints);
        if (numberOfPoints <= 3)
        {
            linesToRemove.push_back(line.id);
        }
    }
    ptr_Storage->transaction([&]()
    {
        for(int lineIdToRemove : linesToRemove)
        {
            ROS_INFO("Removing line with id: %d", lineIdToRemove);
            ptr_Storage->remove_all<DataModel::LinePoint>(where(c(&DataModel::LinePoint::lineId) == lineIdToRemove));
            ptr_Storage->remove_all<DataModel::Line>(where(c(&DataModel::Line::id) == lineIdToRemove));
        }

        return true;
    });
}


struct FitHelper
{
    Point inCarSpace;
    DataModel::LinePoint point;
};

void DoLineAproximationAndSmoothing(std::vector<FitHelper> &pointsToFit, const GeodeticConverter &conv)
{
    ROS_INFO("Aproximating line over points: %zu", pointsToFit.size());
    if (pointsToFit.size() < 4)
    {
        ROS_INFO("Not enough points skiping aproximation.");
        return;
    }
    // Do line aproximation
    Eigen::MatrixXd oXMatrix(pointsToFit.size(), 4);
    Eigen::VectorXd oYMatrix(pointsToFit.size());

    // copy y matrix
    for ( size_t i = 0; i < pointsToFit.size(); i++ )
    {
        oYMatrix(i) = pointsToFit[i].inCarSpace.y;
    }

    // create the X matrix
    for ( size_t nRow = 0; nRow < pointsToFit.size(); nRow++ )
    {
        double nVal = 1.0;
        for ( int nCol = 0; nCol < 4; nCol++ )
        {
            oXMatrix(nRow, nCol) = nVal;
            nVal *= pointsToFit[nRow].inCarSpace.x;
        }
    }

    // transpose X matrix
    auto oXt = oXMatrix.transpose();
    // multiply transposed X matrix with X matrix
    auto oXtX = oXt * oXMatrix;
    // multiply transposed X matrix with Y matrix
    auto oXtY( oXt * oYMatrix );

    Eigen::VectorXd coeff = oXtX.fullPivHouseholderQr().solve(oXtY);

    ROS_INFO("Aproximated line: %f %f %f %f", coeff[0], coeff[1], coeff[2], coeff[3]);
    ptr_Storage->transaction([&]()
    {
        for (auto &pointHelper : pointsToFit)
        {
            double x = pointHelper.inCarSpace.x;
            pointHelper.inCarSpace.y = coeff[3]*x*x*x + coeff[2]*x*x + coeff[1]*x + coeff[0];
            GeoPoint newPoint = conv.carSpaceToGeodetic(pointHelper.inCarSpace);
            pointHelper.point.lat = newPoint.lat;
            pointHelper.point.lon = newPoint.lon;
            pointHelper.point.alt = newPoint.h;

            ROS_INFO("Corecting point: %f %f %f [%f, %f, %f]",
                     pointHelper.inCarSpace.x, pointHelper.inCarSpace.y, pointHelper.inCarSpace.z,
                     pointHelper.point.lon, pointHelper.point.lat, pointHelper.point.alt);
            ptr_Storage->update(pointHelper.point);
        }
        return true;
    });

    // check if error is small
    // remove all old points
    // create new points with 1 meter distance
}

GeodeticConverter CreareNewConverter(const GeoPoint& first, const GeoPoint& second)
{
    // Create goconverter and convert all points in 50 meters range
    double azimuth = GeodeticConverter::calculateAzimuth(first, second);
    CarPosition flatPos;
    flatPos.point = first;
    flatPos.azimuthReal = azimuth;
    return GeodeticConverter(flatPos);
}

void MakeLineSmooth()
{
    ROS_INFO("Smothing lines ...");
    auto allLines = ptr_Storage->get_all<DataModel::Line>();
    for (const auto &line : allLines)
    {
        auto allPoints = ptr_Storage->get_all<DataModel::LinePoint>(where(c(&DataModel::LinePoint::lineId) == line.id));
        ROS_INFO("Processing line: %d with point count: %zu", line.id, allPoints.size());
        GeodeticConverter conv = CreareNewConverter(allPoints[0], allPoints[1]);
        std::vector<FitHelper> pointsToFit;
        Point lastPoint {-1.0, -1.0, -1.0};
        for(std::size_t i = 0; i < allPoints.size() ; ++i)
        {
            DataModel::LinePoint point = allPoints[i];
            Point inCarPoint = conv.geodeticToCarSpace(point);
            ROS_INFO("Analyzing point: %f %f %f [%f, %f, %f]", inCarPoint.x, inCarPoint.y, inCarPoint.z, point.lon, point.lat, point.alt);
            if (inCarPoint.x > 60.0)
            {
                ROS_INFO("Segment detected!");
                // Fit line
                DoLineAproximationAndSmoothing(pointsToFit, conv);

                pointsToFit.clear();
                lastPoint = {-1.0, -1.0, -1.0};
                if (allPoints.size() != (i+1))
                {
                    conv = CreareNewConverter(allPoints[i], allPoints[i+1]);
                    inCarPoint = conv.geodeticToCarSpace(point);
                }
            }

            pointsToFit.push_back({inCarPoint, point});

            lastPoint = inCarPoint;

        }
        DoLineAproximationAndSmoothing(pointsToFit, conv);

    }
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

  ROS_INFO("Start database post processing ...");

  RemoveLinesWith3PointsAndLess();

  MakeLineSmooth();
}
