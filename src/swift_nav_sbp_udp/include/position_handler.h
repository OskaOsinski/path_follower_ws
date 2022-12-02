#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

#include <string>
#include <thread>
#include <memory>
#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <gps_msgs/SystemState.h>
#include <gps_msgs/UTMPosition.h>
#include "libsbp/imu.h"
#include "libsbp/mag.h"
#include "libsbp/logging.h"
#include "libsbp/navigation.h"
#include "libsbp/system.h"
#include "swiftnav_udp_server.h"
#include <GeographicLib/Geodesic.hpp>

class PositionHandler
{
public:
    PositionHandler(ros::NodeHandlePtr publicNode, ros::NodeHandlePtr privateNode);

    void PixiLogMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiImuMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiMagMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiPosSingleMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiPosMultipleMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiVelMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiAgeOfCorrectionMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiDGNSSStatusMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);
    void PixiBaselineNEDVelMsg(SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender);



    t_MsgCallback getCallbackHandler();

private:
    void FrontPixiCallback(const msg_pos_llh_t &msg);
    void TailPixiCallback(const msg_pos_llh_t &msg);
    void SendSystemStateMsg();

    ros::NodeHandlePtr privateNode;
    ros::NodeHandlePtr publicNode;

    boost::asio::ip::address frontPixiIp;

    ros::Publisher pub_Imu;
    ros::Publisher pub_NavSatFix;
    ros::Publisher pub_NavSatFixTail;
    ros::Publisher pub_MagneticField;
    ros::Publisher pub_SystemState;
    ros::Publisher pub_UTMPosition;
    ros::Publisher pub_BaseStationInfoFront;
    ros::Publisher pub_BaseStationInfoTail;

    t_mapIdToMsgCallback callbackMap;
    bool detectedTwoPixi = false;
    boost::asio::ip::address firstPixi;
    boost::asio::ip::address secondPixi;
    msg_age_corrections_t firstPixiAgeOfCorrection;
    msg_age_corrections_t secondPixiAgeOfCorrection;
    msg_baseline_ned_t firstPixiBaselineNED;
    msg_baseline_ned_t secondPixiBaselineNED;
    msg_vel_ned_t firstPixiVelNED;

    std::list<std::pair<ros::Time, msg_pos_llh_t>> frontPixiMsg;
    std::list<std::pair<ros::Time, msg_pos_llh_t>> tailPixiMsg;
    bool initTime = true;
    double headingOffset = 0.0;
    bool fixAsOmniStar = false;

    const GeographicLib::Geodesic* ptrGeod;
};

#endif // POSITION_HANDLER_H
