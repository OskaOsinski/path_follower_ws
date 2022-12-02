#ifndef swift_nav_logic_h
#define swift_nav_logic_h

#include <QObject>
#include <QVector>
#include <QJsonObject>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "gps_msgs/SystemState.h"
#include "gps_msgs/UTMPosition.h"
#include "sensor_msgs/NavSatFix.h"
#include "swift_nav_sbp_udp_msgs/BaseStationInfo.h"
#include "kml_parser_publisher/reference_point.h"
#include "kml_parser_publisher/logic_reference_point.h"
#include "gps_msgs/SystemStatus.h"

#include <std_msgs/String.h>
#include "logic_swift_nav/AdvertisePath.h"
#include "logic_swift_nav/StartPathRecord.h"
#include "logic_swift_nav/StopPathRecord.h"

#pragma once

class SwiftNavLogic
{
public:
    SwiftNavLogic();
    ~SwiftNavLogic();

private:
    void callReferencePointService();
    void initRosCommunication();
    void subscribeToTopics();
    void initAdvertisers();
    void initCarMarker();
    void onRosSystemState(gps_msgs::SystemStateConstPtr ptrMsg);
    void onRosUTM(gps_msgs::UTMPositionConstPtr ptrMsg);
    void newPath(std::string msg);

    void sendActualPositionAndDrivePath(double refX, double refY);
    void sendActualCarMarker(const geometry_msgs::Point& actRearPose, const tf2::Quaternion& quat);
    void transformBroadcast(const geometry_msgs::Point& actRearPose, std::string parentId, std::string childId, double ms);
    void advertisePathCallback(const ros::TimerEvent&);
    void onRecordPathPushButtonToggle(bool state);

    bool startPathRecordServiceHandler(logic_swift_nav::StartPathRecordRequest& req, logic_swift_nav::StartPathRecordResponse& res);
    bool stopPathRecordServiceHandler(logic_swift_nav::StopPathRecordRequest& req, logic_swift_nav::StopPathRecordResponse& res);
    bool advertisePathServiceHandler(logic_swift_nav::AdvertisePathRequest& req, logic_swift_nav::AdvertisePathResponse& res);

    ros::NodeHandle nodeHandler;

    ros::Subscriber subgpsState;
    ros::Subscriber subUtmPos;

    gps_msgs::UTMPosition actualPositionUtm;
    gps_msgs::SystemState actualPositionSystem;

    struct RefPointNavSat {
        double longit;
        double latit;
        double altit;
    } refPointNavSat;

    double refPointXFromService = 0.0;
    double refPointYFromService = 0.0;

    double gpsOffsetXaxis = 0.0;
    double gpsOffsetYaxis = 0.0;
    double carAxisShift = 0.0;

    double lidarFrontOffsetX = 0.0;
    double lidarFrontOffsetY = 0.0;
    double lidarFrontOffsetZ = 0.0;
    double lidarFrontOffsetYaw = 0.0;

    double lidarRearOffsetX = 0.0;
    double lidarRearOffsetY = 0.0;
    double lidarRearOffsetZ = 0.0;
    double lidarRearOffsetYaw = 0.0;

    bool recordingStartedFlag = false;
    QVector<gps_msgs::SystemState> recordedSystemStateMsgs;
    QVector<gps_msgs::UTMPosition> recordedUTMMsgs;
    QVector<gps_msgs::UTMPosition> vehiclePathUTMMsgs;
    QString waypointFileName;

    QString saveWaypointFileName;
    QJsonObject waypointFromFile;
    ros::Timer advertiesePathTimer;

    ros::Publisher carMarkerPub;
    visualization_msgs::Marker carMarker;

    ros::Publisher vehicleDrivePathPub;
    ros::Publisher vehiclePosePub;
    ros::Publisher globalPathPub;

    ros::Publisher currentPosPub;
    ros::Publisher enuHeadingPub;

    ros::ServiceClient clientGetReferencePoint;
    ros::ServiceClient clientSetReferencePoint;
    tf2_ros::TransformBroadcaster br;

    ros::ServiceServer serviceStartPathRecord;
    ros::ServiceServer serviceStopPathRecord;
    ros::ServiceServer serviceAdvertisePath;

};
#endif //swift_nav_logic_h
