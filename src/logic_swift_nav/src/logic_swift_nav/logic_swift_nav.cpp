#include <cmath>
#include <limits>
#include <stdlib.h> 

#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <GeographicLib/UTMUPS.hpp>

#include "logic_swift_nav/logic_swift_nav.h"

#include <QMessageBox>
#include <QDir>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDebug>

SwiftNavLogic::SwiftNavLogic()
{
    initRosCommunication();
}

void SwiftNavLogic::initRosCommunication()
{
    nodeHandler.param<double>("/vehicle_data/sensors/gps/offset_from_rear_axis_x", gpsOffsetXaxis, - 0.35);
    nodeHandler.param<double>("/vehicle_data/sensors/gps/offset_from_rear_axis_y", gpsOffsetYaxis, 0.0);
    nodeHandler.param<double>("/vehicle_data/kia_niro/rear_axis_to_rear", carAxisShift, 0.785);

    nodeHandler.param<double>("/vehicle_data/sensors/front_lidar/offset_from_rear_axis_x", lidarFrontOffsetX, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/front_lidar/offset_from_rear_axis_y", lidarFrontOffsetY, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/front_lidar/offset_from_rear_axis_z", lidarFrontOffsetZ, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/front_lidar/yaw_angle", lidarFrontOffsetYaw, 0.0);

    nodeHandler.param<double>("/vehicle_data/sensors/rear_lidar/offset_from_rear_axis_x", lidarRearOffsetX, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/rear_lidar/offset_from_rear_axis_y", lidarRearOffsetY, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/rear_lidar/offset_from_rear_axis_z", lidarRearOffsetZ, 0.0);
    nodeHandler.param<double>("/vehicle_data/sensors/rear_lidar/yaw_angle", lidarRearOffsetYaw, 0.0);

    initAdvertisers();
    initCarMarker();
    
    subscribeToTopics();

    advertiesePathTimer = nodeHandler.createTimer(ros::Duration(0.05), &SwiftNavLogic::advertisePathCallback, this);

    clientGetReferencePoint = nodeHandler.serviceClient<kml_parser_publisher::logic_reference_point>("swiftnav_new_ref_point");
    clientSetReferencePoint = nodeHandler.serviceClient<kml_parser_publisher::reference_point>("kml_new_ref_point");

    // callReferencePointService();

    serviceStartPathRecord = nodeHandler.advertiseService("start_path_record", &SwiftNavLogic::startPathRecordServiceHandler, this);
    serviceStopPathRecord = nodeHandler.advertiseService("stop_path_record", &SwiftNavLogic::stopPathRecordServiceHandler, this);
    serviceAdvertisePath = nodeHandler.advertiseService("advertise_path", &SwiftNavLogic::advertisePathServiceHandler, this);

}

void SwiftNavLogic::callReferencePointService()
{
    ros::service::waitForService("swiftnav_new_ref_point", 5000);  //this is optional
    kml_parser_publisher::logic_reference_point srv;
    if (clientGetReferencePoint.call(srv)) {
        int zone;
        bool northp;
        GeographicLib::UTMUPS::Forward(srv.response.latitude, srv.response.longitude,
                zone, northp, refPointXFromService, refPointYFromService);
        std::string zonestr = GeographicLib::UTMUPS::EncodeZone(zone, northp);

        ROS_INFO("Service returned refpoint: wgs84[lat, lon] = [%f,%f] "
                 "utm[x, y, zone, isNort] = [%f, %f, %d (%s), %d] ",
                 srv.response.latitude, srv.response.longitude,
                 refPointXFromService, refPointYFromService, zone, zonestr.c_str(), northp);
    }
    else
    {
        ROS_INFO("Failed to call service for 5 times. "
                  "WGS84 reference point was not read from swiftnav_new_ref_point. "
                  "Possibly laptop visualization still not running while system starts");
    }

}

void SwiftNavLogic::subscribeToTopics()
{
    subgpsState = nodeHandler.subscribe("gps_state", 1, &SwiftNavLogic::onRosSystemState, this);
    subUtmPos = nodeHandler.subscribe("utm_position", 1, &SwiftNavLogic::onRosUTM, this);
}

void SwiftNavLogic::initAdvertisers()
{
    vehicleDrivePathPub = nodeHandler.advertise<nav_msgs::Path> ("vehicle_drive_path", 1, true);
    vehiclePosePub = nodeHandler.advertise<geometry_msgs::PoseStamped> ("vehicle_pose", 1);
    enuHeadingPub = nodeHandler.advertise<std_msgs::Float64>("vehicle_enu_heading", 1);
    globalPathPub = nodeHandler.advertise<nav_msgs::Path> ("path_to_follow", 1);
    carMarkerPub = nodeHandler.advertise<visualization_msgs::Marker> ("car_marker", 1, true);
}

void SwiftNavLogic::initCarMarker()
{
    carMarker.header.frame_id = "car_rear_axel";
    carMarker.ns = "car_rear_axel_marker";
    carMarker.id = 0;
    carMarker.type = visualization_msgs::Marker::CUBE;
    carMarker.action = visualization_msgs::Marker::ADD;

    nodeHandler.param<double>(
               "/vehicle_data/kia_niro/length", carMarker.scale.x, 4.355);
    nodeHandler.param<double>(
           "/vehicle_data/kia_niro/width", carMarker.scale.y, 1.805);
    nodeHandler.param<double>(
           "/vehicle_data/kia_niro/height", carMarker.scale.z, 1.545);

    carMarker.color.a = 1.0;
    carMarker.color.r = 0.94;
    carMarker.color.g = 0.03137;
    carMarker.color.b = 0.49804;
}

void SwiftNavLogic::onRosSystemState(gps_msgs::SystemStateConstPtr ptrMsg)
{
    actualPositionSystem = *ptrMsg;
    if(recordingStartedFlag) {
        recordedSystemStateMsgs.push_back(*ptrMsg);
    }
}

void SwiftNavLogic::onRosUTM(gps_msgs::UTMPositionConstPtr ptrMsg)
{
    actualPositionUtm = *ptrMsg;
    if(recordingStartedFlag) {
        recordedUTMMsgs.push_back(*ptrMsg);
    }
    vehiclePathUTMMsgs.push_back(*ptrMsg);
    if (vehiclePathUTMMsgs.size() > 1000) {
        vehiclePathUTMMsgs.pop_front();
    }

}

bool SwiftNavLogic::startPathRecordServiceHandler(logic_swift_nav::StartPathRecordRequest &req, logic_swift_nav::StartPathRecordResponse &res)
{
    ROS_INFO("SwiftNavLogic => Start recording");

    recordingStartedFlag = true;

    refPointNavSat.latit  = actualPositionSystem.nav_sat_fix.latitude;
    refPointNavSat.longit = actualPositionSystem.nav_sat_fix.longitude;
    refPointNavSat.altit  = actualPositionSystem.nav_sat_fix.height;

    std::string path = ros::package::getPath("rqt_swift_nav") + req.pathFileName;
    saveWaypointFileName = QString::fromStdString(path);
    recordedSystemStateMsgs.clear();
    res.ifStarted = true;
    return true;
}

bool SwiftNavLogic::stopPathRecordServiceHandler(
    logic_swift_nav::StopPathRecordRequest &req, logic_swift_nav::StopPathRecordResponse &res)
{
    ROS_INFO("SwiftNavLogic => Stop recording");

    recordingStartedFlag = false;

    for(const auto &msg : recordedSystemStateMsgs)
    {
        if (msg.filter_status.GNSSFixStatus !=
                gps_msgs::FilterStatus::FIX_OMNISTAR_GNSS_FIX )
        {
            ROS_ERROR("Lost fix during recording!");
            res.status = logic_swift_nav::StopPathRecordResponse::STATUS_LOST_FIX;
            return true;
        }
    }

    double firstZoneNumber = recordedUTMMsgs.first().utm_zone_number;
    char firstZoneChar = recordedUTMMsgs.first().utm_zone_character;

    for(const auto &msg : recordedUTMMsgs)
    {
        if (firstZoneNumber != msg.utm_zone_number ||
            firstZoneChar != msg.utm_zone_character)
        {
            ROS_ERROR("Detected UTM zone border!");
            res.status = logic_swift_nav::StopPathRecordResponse::STATUS_LOST_UTM_ZONE_BORDER;
            return true;
        }
    }
    double xRef = recordedUTMMsgs.first().utm_easting;
    double yRef = recordedUTMMsgs.first().utm_northing;
    QJsonObject mainObject;
    mainObject["utm_ref_x"] = xRef;
    mainObject["utm_ref_y"] = yRef;

    mainObject["gps_ref_lat"] = refPointNavSat.latit;
    mainObject["gps_ref_lon"] = refPointNavSat.longit;
    mainObject["gps_ref_alt"] = refPointNavSat.altit;

    QJsonArray utmPosArray;
    for(int i = 1; i < recordedUTMMsgs.size() ; ++i)
    {
        double xRel = recordedUTMMsgs[i].utm_easting - xRef;
        double yRel = recordedUTMMsgs[i].utm_northing - yRef;
        double distance = std::sqrt((xRel)*(xRel) + (yRef)*(yRef));
        if (distance > 0.5) {
            QJsonObject position;
            position["x"] = xRel;
            position["y"] = yRel;
            utmPosArray.append(position);
        }

    }
    mainObject["waypoints"] = utmPosArray;
    QJsonDocument saveDoc(mainObject);
    QFile waypointFile(saveWaypointFileName);

    if (!waypointFile.open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open save file.");
        res.status = logic_swift_nav::StopPathRecordResponse::STATUS_INVALID_FILE;
        return true;
    }
    QByteArray waypoints = saveDoc.toJson();
    res.status = logic_swift_nav::StopPathRecordResponse::STATUS_OK;
    res.waypoints_data = QString::fromUtf8(waypoints).toStdString();
    waypointFile.write(waypoints);
    waypointFromFile = mainObject;
    recordedUTMMsgs.clear();

    return true;
}

bool SwiftNavLogic::advertisePathServiceHandler(logic_swift_nav::AdvertisePathRequest &req, logic_swift_nav::AdvertisePathResponse &res)
{

    std::string path = ros::package::getPath("rqt_swift_nav") + req.pathFileName;
    saveWaypointFileName = QString::fromStdString(path);
    newPath(path);

    return true;
}

void SwiftNavLogic::newPath(std::string position)
{
    waypointFileName = QString::fromStdString(position);
    QFile waypointFile(waypointFileName);

    if (!waypointFile.open(QIODevice::ReadOnly)) {
        ROS_WARN("Couldn't open save file.");
        return;
    }

    QByteArray saveData = waypointFile.readAll();
    QJsonDocument saveWaypoint(QJsonDocument::fromJson(saveData));
    waypointFromFile = saveWaypoint.object();

    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    QJsonArray waypointArray = waypointFromFile["waypoints"].toArray();

    for(int i = 0 ; i < waypointArray.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = waypointArray.at(i).toObject()["x"].toDouble();
        pose.pose.position.y = waypointArray.at(i).toObject()["y"].toDouble();
        msg.poses.push_back(pose);
    }

    if(msg.poses.size()>0) {
        double refLong = waypointFromFile["gps_ref_lon"].toDouble();
        double refLat  = waypointFromFile["gps_ref_lat"].toDouble();
        double refAlt  = waypointFromFile["gps_ref_alt"].toDouble();

        kml_parser_publisher::reference_point srv;
        srv.request.longitude = refLong;
        srv.request.latitude =  refLat;
        srv.request.altitude =  refAlt;

        srv.request.altitude =  0;
        ROS_INFO("Position: %f, %f, %f", srv.request.longitude, srv.request.latitude, srv.request.longitude);
        if (clientSetReferencePoint.call(srv)) {
            ROS_INFO("Kml visualization updated");
        } else {
            ROS_ERROR("Failed to call service kml_new_ref_point");
        }
    }

    globalPathPub.publish(msg);
}

void SwiftNavLogic::sendActualPositionAndDrivePath(double refX, double refY)
{
    double actualGpsPositionX = actualPositionUtm.utm_easting - refX;
    double actualGpsPositionY = actualPositionUtm.utm_northing - refY;
    double carEnuHeading = fmod(2.5*M_PI - actualPositionSystem.orientation_rad.heading, 2*M_PI);

    double headingSin = std::sin(carEnuHeading);
    double headingCos = std::cos(carEnuHeading);
    double carRearCenterAxisXoffset = gpsOffsetXaxis * headingCos - gpsOffsetYaxis * headingSin;
    double carRearCenterAxisYoffset = gpsOffsetXaxis * headingSin + gpsOffsetYaxis * headingCos;

    double carRearCenterAxisX = actualGpsPositionX + carRearCenterAxisXoffset;
    double carRearCenterAxisY = actualGpsPositionY + carRearCenterAxisYoffset;
 
    //ROS_INFO("gps offset rear axel: %f %f %f %f", carRearCenterAxisXoffset, carRearCenterAxisYoffset, gpsOffsetXaxis, gpsOffsetYaxis);    

    geometry_msgs::PointStamped msgActualPose;
    msgActualPose.header.frame_id = "map";
    msgActualPose.header.stamp = ros::Time::now();
    msgActualPose.point.x = carRearCenterAxisX;
    msgActualPose.point.y = carRearCenterAxisY;
 //   currentPosPub.publish(msgActualPose);

    std_msgs::Float64 msgEnuHeading;
    msgEnuHeading.data = carEnuHeading;
    enuHeadingPub.publish(msgEnuHeading);


    geometry_msgs::PoseStamped poseStampedWithOrientation;
    poseStampedWithOrientation.header.frame_id = "map";
    poseStampedWithOrientation.header.stamp = ros::Time::now();
    poseStampedWithOrientation.pose.position = msgActualPose.point;

    tf2::Quaternion quat;
    quat.setRPY( 0, 0, msgEnuHeading.data);
    quat.normalize();
    poseStampedWithOrientation.pose.orientation = tf2::toMsg(quat);
    vehiclePosePub.publish(poseStampedWithOrientation);

    transformBroadcast(msgActualPose.point, "map", "car_rear_axel", msgEnuHeading.data);
    geometry_msgs::Point velodyneFrontPose;
    velodyneFrontPose.x = lidarFrontOffsetX;
    velodyneFrontPose.y = lidarFrontOffsetY;
    velodyneFrontPose.z = lidarFrontOffsetZ;
    transformBroadcast(velodyneFrontPose, "car_rear_axel", "velodyne", lidarFrontOffsetYaw);
    
    geometry_msgs::Point velodyneRearPose;
    velodyneRearPose.x = lidarRearOffsetX;
    velodyneRearPose.y = lidarRearOffsetY;
    velodyneRearPose.z = lidarRearOffsetZ;
    transformBroadcast(velodyneRearPose, "car_rear_axel", "velodyne_rear", lidarRearOffsetYaw);

    sendActualCarMarker(msgActualPose.point, quat);

    {
        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        for(int i = 0 ; i < vehiclePathUTMMsgs.size(); ++i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = vehiclePathUTMMsgs[i].utm_easting - refX;
            pose.pose.position.y = vehiclePathUTMMsgs[i].utm_northing - refY;
            msg.poses.push_back(pose);
        }
        vehicleDrivePathPub.publish(msg);
    }
}

void SwiftNavLogic::transformBroadcast( const geometry_msgs::Point& actRearPose,
    std::string parentId, std::string childId, double yaw)
{
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = parentId;
    transformStamped.child_frame_id = childId;
    transformStamped.transform.translation.x = actRearPose.x;
    transformStamped.transform.translation.y = actRearPose.y;
    transformStamped.transform.translation.z = actRearPose.z;
    tf2::Quaternion q;
    //q.setRPY(0, 0, - actualPositionSystem.orientation_rad.heading + M_PI/2.0 + M_PI);
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation = tf2::toMsg(q);
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
}

void SwiftNavLogic::sendActualCarMarker(
    const geometry_msgs::Point& actRearPose, const tf2::Quaternion& quat)
{
    carMarker.pose.position.x = carMarker.scale.x/2.0 - carAxisShift;
    carMarker.pose.position.y = 0;
    carMarker.pose.position.z = carMarker.scale.z / 2.0;
   // carMarker.pose.position = actRearPose;
    //carMarker.pose.orientation = tf2::toMsg(quat);

    carMarker.header.stamp = ros::Time::now();
    carMarkerPub.publish(carMarker);
}

void SwiftNavLogic::advertisePathCallback(const ros::TimerEvent &)
{
    if (
        waypointFromFile.contains("utm_ref_x") &&
        waypointFromFile.contains("utm_ref_y") &&
        waypointFromFile.contains("waypoints"))
    {
        double refX = waypointFromFile["utm_ref_x"].toDouble();
        double refY = waypointFromFile["utm_ref_y"].toDouble();

        sendActualPositionAndDrivePath(refX, refY);

    }
    else
    {
        sendActualPositionAndDrivePath(refPointXFromService, refPointYFromService);
    }
}

SwiftNavLogic::~SwiftNavLogic()
{
    advertiesePathTimer.stop();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "swift_nav_logic");
    SwiftNavLogic logic;

    ros::spin();
}
