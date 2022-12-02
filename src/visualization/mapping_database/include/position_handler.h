#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

#include <functional>
#include <memory>
#include <ros/ros.h>
#include <gps_msgs/SystemState.h>
#include "commonstruct.h"
#include "geodeticconverter.hpp"

class PositionHandler
{
public:
    typedef std::function<void (GeoPoint minPoint, GeoPoint maxPoint)> t_fetchFun;

    void setNewPosition(const gps_msgs::SystemState::ConstPtr &msg);
    void interpolatePosition(const ros::Time &timeStamp);
    bool isValidPosition() {return  m_positionIsValid;}
    std::shared_ptr<GeodeticConverter> getConverter(){return m_ptrActualGeoConverter;}
    void setFetchingFunction(t_fetchFun fetchFun, double fetchDistance, double distanceBetweenFetch);

private:
    gps_msgs::SystemState m_actualPosition;
    double m_speed = 0.0;
    double m_yawRate = 0.0;
    bool m_positionIsValid = false;
    GeoPoint m_lastFetchPosition = {0.0, 0.0, 0.0};
    std::shared_ptr<GeodeticConverter> m_ptrActualGeoConverter;
    t_fetchFun m_fetchFun;
    double m_distanceBetweenFetch = 0.0;
    double m_fetchDistance = 0.0;
    ros::Time lastRecvTime;

};

#endif // POSITION_HANDLER_H
