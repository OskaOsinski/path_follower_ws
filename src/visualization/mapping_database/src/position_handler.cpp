#include "position_handler.h"



void PositionHandler::setNewPosition(const gps_msgs::SystemState::ConstPtr &msg)
{
    if (msg->filter_status.GNSSFixStatus != gps_msgs::FilterStatus::FIX_FTK_FIXED_GNSS_FIX)
    {
        ROS_INFO("Position not fixed dropping frame: %d", msg->header.seq);
        return;
    }

    if (m_positionIsValid && msg->filter_status.GNSSFixStatus == gps_msgs::FilterStatus::FIX_FTK_FIXED_GNSS_FIX )
    {
        double R = 6371e3; // metres
        double latInRad1 = GeodeticConverter::DEG_TO_RAD(m_actualPosition.nav_sat_fix.latitude);
        double latInRad2 = GeodeticConverter::DEG_TO_RAD(msg->nav_sat_fix.latitude);
        double deltaLatInRad = GeodeticConverter::DEG_TO_RAD(msg->nav_sat_fix.latitude - m_actualPosition.nav_sat_fix.latitude);
        double deltaLonInRad = GeodeticConverter::DEG_TO_RAD(msg->nav_sat_fix.longitude - m_actualPosition.nav_sat_fix.longitude);

        double a = std::sin(deltaLatInRad/2.0) * std::sin(deltaLatInRad/2.0) +
                std::cos(latInRad1) * std::cos(latInRad2) *
                std::sin(deltaLonInRad/2.0) * std::sin(deltaLonInRad/2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0-a));

        double distance = R * c;

        ros::Duration timeDiff = msg->header.stamp - m_actualPosition.header.stamp;
        m_speed = distance / timeDiff.toSec();
        m_yawRate = (msg->orientation_rad.heading - m_actualPosition.orientation_rad.heading) / timeDiff.toSec();
        ROS_INFO("Calculated speed=%f m/s and yaw rate=%f rad/s timeDiff=%f s", m_speed, m_yawRate, timeDiff.toSec());
    }

    m_actualPosition = *msg;
    m_positionIsValid = m_actualPosition.filter_status.GNSSFixStatus == gps_msgs::FilterStatus::FIX_FTK_FIXED_GNSS_FIX;

    CarPosition position;
    position.point.lon = m_actualPosition.nav_sat_fix.longitude;
    position.point.lat = m_actualPosition.nav_sat_fix.latitude;
    position.point.h = m_actualPosition.nav_sat_fix.height;
    position.azimuthReal = m_actualPosition.orientation_rad.heading;
    m_ptrActualGeoConverter = std::make_shared<GeodeticConverter>(position);

    lastRecvTime = m_actualPosition.header.stamp;

    if (m_fetchFun)
    {
        Point lastFetchInCarSpace = m_ptrActualGeoConverter->geodeticToCarSpace(m_lastFetchPosition);
        double distanceFromLastFetch = std::sqrt(lastFetchInCarSpace.x * lastFetchInCarSpace.x + lastFetchInCarSpace.y * lastFetchInCarSpace.y);
        if (distanceFromLastFetch > m_distanceBetweenFetch)
        {
            GeoPoint upperLeft = m_ptrActualGeoConverter->carSpaceToGeodetic({m_fetchDistance,m_fetchDistance,0});
            GeoPoint upperRight = m_ptrActualGeoConverter->carSpaceToGeodetic({m_fetchDistance,-m_fetchDistance,0});
            GeoPoint lowerLeft = m_ptrActualGeoConverter->carSpaceToGeodetic({-m_fetchDistance,m_fetchDistance,0});
            GeoPoint lowerRight = m_ptrActualGeoConverter->carSpaceToGeodetic({-m_fetchDistance,-m_fetchDistance,0});

            GeoPoint min = {0.0, 0.0, 0.0};
            GeoPoint max = {0.0, 0.0, 0.0};;
            min.lon = std::min({upperLeft.lon, upperRight.lon, lowerLeft.lon, lowerRight.lon});
            max.lon = std::max({upperLeft.lon, upperRight.lon, lowerLeft.lon, lowerRight.lon});

            min.lat = std::min({upperLeft.lat, upperRight.lat, lowerLeft.lat, lowerRight.lat});
            max.lat = std::max({upperLeft.lat, upperRight.lat, lowerLeft.lat, lowerRight.lat});

            m_fetchFun(min, max);

            m_lastFetchPosition.lon = position.point.lon;
            m_lastFetchPosition.lat = position.point.lat;
            m_lastFetchPosition.h = position.point.h;

        }
    }

}

void PositionHandler::interpolatePosition(const ros::Time &timeStamp)
{
    CarPosition position;
    position.point = m_actualPosition.nav_sat_fix;
    position.azimuthReal = m_actualPosition.orientation_rad.heading;
    GeodeticConverter center(position);

    double timeDiff = (timeStamp - m_actualPosition.header.stamp).toSec();
    Point start = {0.0, 0.0, 0.0};
    start.x += m_speed*timeDiff;

    double rotation = m_yawRate*timeDiff;
    start = GeodeticConverter::rotatePointBy(rotation, start);
    ROS_INFO("Calculated new position [x,y,z] [%f, %f, %f] timeDiff=%f", start.x, start.y, start.z, timeDiff);

    CarPosition finalPosition;
    finalPosition.point = center.carSpaceToGeodetic(start);
    finalPosition.azimuthReal = m_actualPosition.orientation_rad.heading + rotation;
    m_ptrActualGeoConverter = std::make_shared<GeodeticConverter>(finalPosition);

}

void PositionHandler::setFetchingFunction(PositionHandler::t_fetchFun fetchFun, double fetchDistance, double distanceBetweenFetch)
{
    m_fetchFun = fetchFun;
    m_fetchDistance = fetchDistance;
    m_distanceBetweenFetch = distanceBetweenFetch;
}
