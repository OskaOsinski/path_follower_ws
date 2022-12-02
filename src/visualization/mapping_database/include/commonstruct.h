#ifndef COMMONSTRUCT_H
#define COMMONSTRUCT_H

#include <memory>
#include <gps_msgs/NavSatFix.h>


/**
 * @brief The GeoPoint struct represent point in WGS84 space
 */
struct GeoPoint
{
    double lon; /**< longitude */
    double lat; /**< latitude */
    double h;   /**< altitude */

    GeoPoint()
    {
        lon = 0.0;
        lat = 0.0;
        h = 0.0;
    }

    GeoPoint(std::initializer_list<double> list)
    {
        lon = *list.begin();
        lat = *(list.begin()+1);
        h = *(list.begin()+2);
    }

    GeoPoint(const gps_msgs::NavSatFix &nav)
    {
        lon = nav.longitude;
        lat = nav.latitude;
        h = nav.height;
    }
};

struct Point
{
    double x; /**< longitude */
    double y; /**< latitude */
    double z;   /**< altitude */
};

struct CarPosition
{
    GeoPoint point = {0.0, 0.0, 0.0};   /**< Position of car in WGS84 */

    double yaw = 0.0;                   /**< Yaw angle in radian of car point of view in 3D space*/
    double pitch = 0.0;                 /**< Pitch angle in radian of car point of view in 3D space*/
    double roll = 0.0;                  /**< Roll angle in radian of car point of view in 3D space*/

    double azimuthReal = 0.0;           /**< Azimuth on 2D space calculated from yaw, pitch and roll angle in radian*/
};

#endif // COMMONSTRUCT_H
