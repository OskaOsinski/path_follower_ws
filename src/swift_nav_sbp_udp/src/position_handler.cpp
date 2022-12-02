#include "position_handler.h"
#include "swift_nav_sbp_udp_msgs/BaseStationInfo.h"
#include "gps_conv.h"


PositionHandler::PositionHandler(ros::NodeHandlePtr publicNode, ros::NodeHandlePtr privateNode)
{
    this->publicNode = publicNode;
    this->privateNode = privateNode;

    std::string frontIp;
    // TODO: This shoudl be private node insted of public
    privateNode->param<std::string>("front_pixi_ip", frontIp, "");
    privateNode->param<double>("heading_offset", headingOffset, 0);
    privateNode->param<bool>("fix_as_omnistar", fixAsOmniStar, false);
    if (fixAsOmniStar)
    {
        ROS_WARN("RTK Fix status will be set as omnistar fix.");
    }
    ROS_INFO("Front pixi ip: %s with heading offset %f", frontIp.c_str(), headingOffset);
    try
    {
        frontPixiIp = boost::asio::ip::address::from_string(frontIp);
    }
    catch (std::exception exp)
    {
        ROS_ERROR("Invalid front gps ip.");
    }

    pub_Imu = publicNode->advertise<sensor_msgs::Imu>("gps_imu", 1);
    pub_NavSatFix = publicNode->advertise<sensor_msgs::NavSatFix>("gps_left_position", 1);
    pub_NavSatFixTail = publicNode->advertise<sensor_msgs::NavSatFix>("gps_right_position", 1);
    pub_MagneticField = publicNode->advertise<sensor_msgs::MagneticField>("gps_magnetic_compas", 1);
    pub_SystemState = publicNode->advertise<gps_msgs::SystemState>("gps_state", 1);
    pub_UTMPosition = publicNode->advertise<gps_msgs::UTMPosition>("utm_position", 1);
    pub_BaseStationInfoFront = publicNode->advertise<swift_nav_sbp_udp_msgs::BaseStationInfo>("gps_base_status_left", 1);
    pub_BaseStationInfoTail = publicNode->advertise<swift_nav_sbp_udp_msgs::BaseStationInfo>("gps_base_status_right", 1);

    callbackMap[SBP_MSG_IMU_RAW] = std::bind(&PositionHandler::PixiImuMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_MAG_RAW] = std::bind(&PositionHandler::PixiMagMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_LOG] = std::bind(&PositionHandler::PixiLogMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_POS_LLH] = std::bind(&PositionHandler::PixiPosSingleMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_VEL_NED] = std::bind(&PositionHandler::PixiVelMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_AGE_CORRECTIONS] = std::bind(&PositionHandler::PixiAgeOfCorrectionMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_DGNSS_STATUS] = std::bind(&PositionHandler::PixiDGNSSStatusMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    callbackMap[SBP_MSG_BASELINE_NED] = std::bind(&PositionHandler::PixiBaselineNEDVelMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    ptrGeod = &GeographicLib::Geodesic::WGS84();
}

void PositionHandler::PixiLogMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_log_t log;
    memcpy(&log, payload.data(), sizeof(log));

    //ROS_INFO("SBP_MSG_LOG from %s: [%d] %s", sender.to_string().c_str(), log.level, log.text);
}

void PositionHandler::PixiImuMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_imu_raw_t imu;
    memcpy(&imu, payload.data(), sizeof(imu));

    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.linear_acceleration.x = imu.acc_x;
    imuMsg.linear_acceleration.y = imu.acc_y;
    imuMsg.linear_acceleration.z = imu.acc_z;

    imuMsg.angular_velocity.x = imu.gyr_x;
    imuMsg.angular_velocity.y = imu.gyr_y;
    imuMsg.angular_velocity.z = imu.gyr_z;

    pub_Imu.publish(imuMsg);
}

void PositionHandler::PixiMagMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_mag_raw_t mag;
    memcpy(&mag, payload.data(), sizeof(mag));

    sensor_msgs::MagneticField magMsg;
    magMsg.header.stamp = ros::Time::now();
    magMsg.magnetic_field.x = mag.mag_x;
    magMsg.magnetic_field.y = mag.mag_y;
    magMsg.magnetic_field.z = mag.mag_z;

    pub_MagneticField.publish(magMsg);
}

void PositionHandler::PixiPosSingleMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_pos_llh_t pos;
    memcpy(&pos, payload.data(), sizeof(pos));

    if (firstPixi.is_unspecified())
    {
        firstPixi = sender;
    }
    if (firstPixi != sender)
    {
        ROS_INFO("Mutliple pixi detected! Switch to heading mode calculation. First ip: %s second ip: %s", firstPixi.to_string().c_str(), sender.to_string().c_str());
        detectedTwoPixi = true;
        secondPixi = sender;
        callbackMap[SBP_MSG_POS_LLH] = std::bind(&PositionHandler::PixiPosMultipleMsg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        PixiPosMultipleMsg(header, payload, sender);
        return;

    }

    sensor_msgs::NavSatFix navSatFixMsg;
    navSatFixMsg.header.stamp = ros::Time::now();
    navSatFixMsg.longitude = pos.lon;
    navSatFixMsg.latitude = pos.lat;
    navSatFixMsg.altitude = pos.height;
    std::uint8_t fixStatus = pos.flags & 0x07;

    //    Value Description
    //    0 Invalid
    //    1 Single Point Position (SPP)
    //    2 Differential GNSS (DGNSS)
    //    3 Float RTK
    //    4 Fixed RTK
    //    5 Dead Reckoning
    //    6 SBAS Position
    switch (fixStatus)
    {
    case 0 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 1 :
    case 2 :
    case 3 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
    case 4 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
    case 5 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 6 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
    default : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    };

    pub_NavSatFix.publish(navSatFixMsg);

    gps_msgs::SystemState systemStateMsg;
    systemStateMsg.header.stamp = ros::Time::now();
    /*
    systemStateMsg.system_status.raw_status = system_state_packet.system_status.r;	//raw status
        systemStateMsg.system_status.SystemFailure = 		(bool)system_state_packet.system_status.b.system_failure;
        systemStateMsg.system_status.AccelSensorFailure =  (bool)system_state_packet.system_status.b.accelerometer_sensor_failure;
        systemStateMsg.system_status.GyroSensorFailure =   (bool)system_state_packet.system_status.b.gyroscope_sensor_failure;
        systemStateMsg.system_status.MagnetSensorFailure = (bool)system_state_packet.system_status.b.magnetometer_sensor_failure;
        systemStateMsg.system_status.PressureSensorFailure = (bool)system_state_packet.system_status.b.pressure_sensor_failure;
        systemStateMsg.system_status.GNSSFailure = 		  (bool)system_state_packet.system_status.b.gnss_failure;
        systemStateMsg.system_status.AccelOverRange = 		  (bool)system_state_packet.system_status.b.accelerometer_over_range;
        systemStateMsg.system_status.GyroOverRange = 		  (bool)system_state_packet.system_status.b.gyroscope_over_range;
        systemStateMsg.system_status.MagnetOverRange =     (bool)system_state_packet.system_status.b.magnetometer_over_range;
        systemStateMsg.system_status.PressureOverRange =   (bool)system_state_packet.system_status.b.pressure_over_range;
        systemStateMsg.system_status.MinTemperatureAlarm = (bool)system_state_packet.system_status.b.minimum_temperature_alarm;
        systemStateMsg.system_status.MaxTemperatureAlarm = (bool)system_state_packet.system_status.b.maximum_temperature_alarm;
        systemStateMsg.system_status.LowVoltageAlarm = 		(bool)system_state_packet.system_status.b.low_voltage_alarm;
        systemStateMsg.system_status.HighVoltageAlarm = 		(bool)system_state_packet.system_status.b.high_voltage_alarm;
        systemStateMsg.system_status.GNSSAntennaShortCircuit = (bool)system_state_packet.system_status.b.gnss_antenna_disconnected;
        systemStateMsg.system_status.DataOutputOverflowAlarm = (bool)system_state_packet.system_status.b.serial_port_overflow_alarm;
    //system_state::filter_status
    systemStateMsg.filter_status.raw_status = 	system_state_packet.filter_status.r;
        systemStateMsg.filter_status.OrientationFilterInitialized = (bool)system_state_packet.filter_status.b.orientation_filter_initialised;
        systemStateMsg.filter_status.NavigationFilterInitialized =  (bool)system_state_packet.filter_status.b.ins_filter_initialised;
        systemStateMsg.filter_status.HeadingInitialized = (bool)system_state_packet.filter_status.b.heading_initialised;
        systemStateMsg.filter_status.UTCTimeInitialized = (bool)system_state_packet.filter_status.b.utc_time_initialised;
        systemStateMsg.filter_status.GNSSFixStatus = system_state_packet.filter_status.b.gnss_fix_type;
        systemStateMsg.filter_status.Event1Occured = (bool)system_state_packet.filter_status.b.event1_flag;
        systemStateMsg.filter_status.Event2Occured = (bool)system_state_packet.filter_status.b.event2_flag;
        systemStateMsg.filter_status.InternalGNSSEnabled = 	 (bool)system_state_packet.filter_status.b.internal_gnss_enabled;
        systemStateMsg.filter_status.DualAntennaHeadingActive = (bool)system_state_packet.filter_status.b.dual_antenna_heading_active;
        systemStateMsg.filter_status.VelocityHeadingEnabled = 	   (bool)system_state_packet.filter_status.b.velocity_heading_enabled;
        systemStateMsg.filter_status.AtmosphericAltitudeEnabled = (bool)system_state_packet.filter_status.b.atmospheric_altitude_enabled;
        systemStateMsg.filter_status.ExternalPositionActive = (bool)system_state_packet.filter_status.b.external_position_active;
        systemStateMsg.filter_status.ExternalVelocityActive = (bool)system_state_packet.filter_status.b.external_velocity_active;
        systemStateMsg.filter_status.ExternalHeadingActive =  (bool)system_state_packet.filter_status.b.external_heading_active;
    systemStateMsg.unix_time_seconds = system_state_packet.unix_time_seconds;
    systemStateMsg.microseconds = system_state_packet.microseconds;
    systemStateMsg.nav_sat_fix.latitude =  system_state_packet.latitude * RADIANS_TO_DEGREES;
    systemStateMsg.nav_sat_fix.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
    systemStateMsg.nav_sat_fix.height =    system_state_packet.height;
    systemStateMsg.velocity_ned.velocity_north = system_state_packet.velocity[0];
    systemStateMsg.velocity_ned.velocity_east =  system_state_packet.velocity[1];
    systemStateMsg.velocity_ned.velocity_down =  system_state_packet.velocity[2];
    systemStateMsg.body_acceleration.accel_north = system_state_packet.body_acceleration[0];
    systemStateMsg.body_acceleration.accel_east =  system_state_packet.body_acceleration[1];
    systemStateMsg.body_acceleration.accel_down =  system_state_packet.body_acceleration[2];
    systemStateMsg.gforce = system_state_packet.g_force;
    systemStateMsg.orientation_rad.roll =    system_state_packet.orientation[0];
    systemStateMsg.orientation_rad.pitch =   system_state_packet.orientation[1];
    systemStateMsg.orientation_rad.heading = system_state_packet.orientation[2];
    systemStateMsg.angular_velocity_rads.x = system_state_packet.angular_velocity[0];
    systemStateMsg.angular_velocity_rads.y = system_state_packet.angular_velocity[1];
    systemStateMsg.angular_velocity_rads.z = system_state_packet.angular_velocity[2];
    systemStateMsg.sat_fix_std_dev.latitude = system_state_packet.standard_deviation[0];
    systemStateMsg.sat_fix_std_dev.longitude = system_state_packet.standard_deviation[1];
    systemStateMsg.sat_fix_std_dev.height =   system_state_packet.standard_deviation[2];
    */

    systemStateMsg.nav_sat_fix.longitude = static_cast<float>( pos.lon );
    systemStateMsg.nav_sat_fix.latitude = static_cast<float>( pos.lat );
    systemStateMsg.nav_sat_fix.height =  static_cast<float>( pos.height );
    systemStateMsg.velocity_ned.velocity_north = firstPixiVelNED.n;
    systemStateMsg.velocity_ned.velocity_east =  firstPixiVelNED.e;
    systemStateMsg.velocity_ned.velocity_down =  firstPixiVelNED.d;
    //    Value Description
    //    0 Invalid
    //    1 Single Point Position (SPP)
    //    2 Differential GNSS (DGNSS)
    //    3 Float RTK
    //    4 Fixed RTK
    //    5 Dead Reckoning
    //    6 SBAS Position
    switch (fixStatus)
    {
    case 0 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX; break;
    case 1 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_2D_GNSS_FIX; break;
    case 2 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_3D_GNSS_FIX; break;
    case 3 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_RTK_FLOAT_GNSS_FIX; break;
    case 4 : systemStateMsg.filter_status.GNSSFixStatus = fixAsOmniStar ? gps_msgs::FilterStatus::FIX_OMNISTAR_GNSS_FIX : gps_msgs::FilterStatus::FIX_FTK_FIXED_GNSS_FIX; break;
    case 5 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX; break;
    case 6 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_SBAS_GNSS_FIX; break;
    default : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX;
    };

    pub_SystemState.publish(systemStateMsg);

    UTMCoords utmConvert(navSatFixMsg);
    gps_msgs::UTMPosition utmMsg;
    utmMsg.header.stamp = ros::Time::now();
    utmMsg.utm_easting = utmConvert.getX();
    utmMsg.utm_northing = utmConvert.getY();
    utmMsg.utm_height = utmConvert.getZ();
    utmMsg.utm_zone_number = utmConvert.getZone();
    utmMsg.utm_zone_character = utmConvert.getCentralMeridian();

    pub_UTMPosition.publish(utmMsg);
}

void PositionHandler::PixiPosMultipleMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_pos_llh_t pos;
    memcpy(&pos, payload.data(), sizeof(pos));

    if (firstPixi != sender && secondPixi != sender)
    {
        ROS_ERROR("Thrid pixi device detected! Ignoring results from this device.");
        return;
    }

    if (sender == frontPixiIp)
    {
        FrontPixiCallback(pos);
    }
    else
    {
        TailPixiCallback(pos);
    }
}

void PositionHandler::PixiVelMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_vel_ned_t vel;
    memcpy(&vel, payload.data(), sizeof(vel));
    if (firstPixi == sender)
    {
        firstPixiVelNED = vel;
    }
}

void PositionHandler::PixiAgeOfCorrectionMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    if (sender == firstPixi)
    {
        memcpy(&firstPixiAgeOfCorrection, payload.data(), sizeof(firstPixiAgeOfCorrection));
    }
    else
    {
        memcpy(&secondPixiAgeOfCorrection, payload.data(), sizeof(secondPixiAgeOfCorrection));
    }
}

void PositionHandler::PixiDGNSSStatusMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    msg_dgnss_status_t dgnssStatus;
    memcpy(&dgnssStatus, payload.data(), sizeof(dgnssStatus));

    swift_nav_sbp_udp_msgs::BaseStationInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.signal_latency = dgnssStatus.latency * 0.01;


    if (secondPixi.is_unspecified() || sender == frontPixiIp)
    {
        msg.age_of_correction = firstPixiAgeOfCorrection.age * 0.1;
        msg.distance_from_base = std::sqrt(firstPixiBaselineNED.n * firstPixiBaselineNED.n + firstPixiBaselineNED.e * firstPixiBaselineNED.e + firstPixiBaselineNED.d * firstPixiBaselineNED.d);
        pub_BaseStationInfoFront.publish(msg);
    }
    else
    {
        msg.age_of_correction = secondPixiAgeOfCorrection.age * 0.1;
        msg.distance_from_base = std::sqrt(secondPixiBaselineNED.n * secondPixiBaselineNED.n + secondPixiBaselineNED.e * secondPixiBaselineNED.e + secondPixiBaselineNED.d * secondPixiBaselineNED.d);
        pub_BaseStationInfoTail.publish(msg);
    }
}

void PositionHandler::PixiBaselineNEDVelMsg(SbpMsgHeader header, std::vector<uint8_t> payload, boost::asio::ip::address sender)
{
    if (sender == firstPixi)
    {
        memcpy(&firstPixiBaselineNED, payload.data(), sizeof(firstPixiBaselineNED));
    }
    else
    {
        memcpy(&secondPixiBaselineNED, payload.data(), sizeof(secondPixiBaselineNED));
    }
}

t_MsgCallback PositionHandler::getCallbackHandler()
{
    return [this](SbpMsgHeader header, std::vector<std::uint8_t> payload, boost::asio::ip::address sender)
    {
        if (callbackMap.count(header.MsgType))
        {
            callbackMap[header.MsgType](header, payload, sender);
        }
    };
}

void PositionHandler::FrontPixiCallback(const msg_pos_llh_t &msg)
{
    frontPixiMsg.push_back({ros::Time::now(), msg});

    if (tailPixiMsg.size() > 0)
    {
        SendSystemStateMsg();
    }
}

void PositionHandler::TailPixiCallback(const msg_pos_llh_t &msg)
{
    tailPixiMsg.push_back({ros::Time::now(), msg});

    if (frontPixiMsg.size() > 0)
    {
        SendSystemStateMsg();
    }
}

void PositionHandler::SendSystemStateMsg()
{
    ros::Duration timeShift = frontPixiMsg.front().first - tailPixiMsg.front().first;

    if (initTime)
    {

       if (std::fabs(timeShift.toSec()) > 0.08)
       {
           // ROS_INFO("Timeshift between first frames: %f skiping to old frame.", timeShift.toSec());
           if (timeShift.toSec() > 0)
           {
               tailPixiMsg.pop_front();
           }
           else
           {
               frontPixiMsg.pop_front();
           }
           return;
       }
       initTime = false;
    }

    if (std::fabs(timeShift.toSec()) > 0.2)
    {
        // ROS_INFO("Timeshift between frames: %f skiping all to old frame.", timeShift.toSec());
        ros::Duration currentTimeShift = timeShift;
        if (timeShift.toSec() > 0)
        {
            do
            {
                // ROS_INFO("Timeshift between frames: %f skiping frame pop tail %zu.", currentTimeShift.toSec(), tailPixiMsg.size());
                tailPixiMsg.pop_front();
                if (tailPixiMsg.size() == 0)
                {
                    return;
                }
                currentTimeShift = frontPixiMsg.front().first - tailPixiMsg.front().first;
            } while(currentTimeShift.toSec() > 0.08);

        }
        else
        {
            do
            {
                //ROS_INFO("Timeshift between frames: %f skiping frame pop front %zu.", currentTimeShift.toSec(), frontPixiMsg.size());
                frontPixiMsg.pop_front();
                if (frontPixiMsg.size() == 0)
                {
                    return;
                }
                currentTimeShift = frontPixiMsg.front().first - tailPixiMsg.front().first;
            } while(currentTimeShift.toSec() < 0.08);
        }
        return;
    }




    msg_pos_llh_t frontPos = frontPixiMsg.front().second;
    frontPixiMsg.pop_front();
    msg_pos_llh_t tailPos = tailPixiMsg.front().second;
    tailPixiMsg.pop_front();

    double azi1, azi2, s12;
    ptrGeod->Inverse(frontPos.lat, frontPos.lon, tailPos.lat, tailPos.lon, s12, azi1, azi2);

    double middleLat, middleLon;
    ptrGeod->Direct(frontPos.lat, frontPos.lon, azi1, s12*0.5, middleLat, middleLon, azi2);

    sensor_msgs::NavSatFix navSatFixMsg;
    navSatFixMsg.header.stamp = ros::Time::now();
    navSatFixMsg.longitude = middleLon;
    navSatFixMsg.latitude = middleLat;
    navSatFixMsg.altitude = frontPos.height;
    //    Value Description
    //    0 Invalid
    //    1 Single Point Position (SPP)
    //    2 Differential GNSS (DGNSS)
    //    3 Float RTK
    //    4 Fixed RTK
    //    5 Dead Reckoning
    //    6 SBAS Position
    std::uint8_t fixStatus = (frontPos.flags & 0x07);
    switch (fixStatus)
    {
    case 0 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 1 :
    case 2 :
    case 3 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
    case 4 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
    case 5 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 6 : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
    default : navSatFixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    };
    navSatFixMsg.position_covariance[0] = frontPos.h_accuracy;
    navSatFixMsg.position_covariance[4] = frontPos.v_accuracy;

    pub_NavSatFix.publish(navSatFixMsg);

    sensor_msgs::NavSatFix navSatFixTailMsg;
    navSatFixTailMsg.header.stamp = ros::Time::now();
    navSatFixTailMsg.longitude = tailPos.lon;
    navSatFixTailMsg.latitude = tailPos.lat;
    navSatFixTailMsg.altitude = tailPos.height;
    //    Value Description
    //    0 Invalid
    //    1 Single Point Position (SPP)
    //    2 Differential GNSS (DGNSS)
    //    3 Float RTK
    //    4 Fixed RTK
    //    5 Dead Reckoning
    //    6 SBAS Position
    std::uint8_t fixTailStatus = (tailPos.flags & 0x07);
    switch (fixTailStatus)
    {
    case 0 : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 1 :
    case 2 :
    case 3 : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
    case 4 : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
    case 5 : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    case 6 : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
    default : navSatFixTailMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    };
    navSatFixTailMsg.position_covariance[0] = tailPos.h_accuracy;
    navSatFixTailMsg.position_covariance[4] = tailPos.v_accuracy;

    pub_NavSatFixTail.publish(navSatFixTailMsg);



    gps_msgs::SystemState systemStateMsg;
    systemStateMsg.header.stamp = ros::Time::now();
    /*
    systemStateMsg.system_status.raw_status = system_state_packet.system_status.r;	//raw status
        systemStateMsg.system_status.SystemFailure = 		(bool)system_state_packet.system_status.b.system_failure;
        systemStateMsg.system_status.AccelSensorFailure =  (bool)system_state_packet.system_status.b.accelerometer_sensor_failure;
        systemStateMsg.system_status.GyroSensorFailure =   (bool)system_state_packet.system_status.b.gyroscope_sensor_failure;
        systemStateMsg.system_status.MagnetSensorFailure = (bool)system_state_packet.system_status.b.magnetometer_sensor_failure;
        systemStateMsg.system_status.PressureSensorFailure = (bool)system_state_packet.system_status.b.pressure_sensor_failure;
        systemStateMsg.system_status.GNSSFailure = 		  (bool)system_state_packet.system_status.b.gnss_failure;
        systemStateMsg.system_status.AccelOverRange = 		  (bool)system_state_packet.system_status.b.accelerometer_over_range;
        systemStateMsg.system_status.GyroOverRange = 		  (bool)system_state_packet.system_status.b.gyroscope_over_range;
        systemStateMsg.system_status.MagnetOverRange =     (bool)system_state_packet.system_status.b.magnetometer_over_range;
        systemStateMsg.system_status.PressureOverRange =   (bool)system_state_packet.system_status.b.pressure_over_range;
        systemStateMsg.system_status.MinTemperatureAlarm = (bool)system_state_packet.system_status.b.minimum_temperature_alarm;
        systemStateMsg.system_status.MaxTemperatureAlarm = (bool)system_state_packet.system_status.b.maximum_temperature_alarm;
        systemStateMsg.system_status.LowVoltageAlarm = 		(bool)system_state_packet.system_status.b.low_voltage_alarm;
        systemStateMsg.system_status.HighVoltageAlarm = 		(bool)system_state_packet.system_status.b.high_voltage_alarm;
        systemStateMsg.system_status.GNSSAntennaShortCircuit = (bool)system_state_packet.system_status.b.gnss_antenna_disconnected;
        systemStateMsg.system_status.DataOutputOverflowAlarm = (bool)system_state_packet.system_status.b.serial_port_overflow_alarm;
    //system_state::filter_status
    systemStateMsg.filter_status.raw_status = 	system_state_packet.filter_status.r;
        systemStateMsg.filter_status.OrientationFilterInitialized = (bool)system_state_packet.filter_status.b.orientation_filter_initialised;
        systemStateMsg.filter_status.NavigationFilterInitialized =  (bool)system_state_packet.filter_status.b.ins_filter_initialised;
        systemStateMsg.filter_status.HeadingInitialized = (bool)system_state_packet.filter_status.b.heading_initialised;
        systemStateMsg.filter_status.UTCTimeInitialized = (bool)system_state_packet.filter_status.b.utc_time_initialised;
        systemStateMsg.filter_status.GNSSFixStatus = system_state_packet.filter_status.b.gnss_fix_type;
        systemStateMsg.filter_status.Event1Occured = (bool)system_state_packet.filter_status.b.event1_flag;
        systemStateMsg.filter_status.Event2Occured = (bool)system_state_packet.filter_status.b.event2_flag;
        systemStateMsg.filter_status.InternalGNSSEnabled = 	 (bool)system_state_packet.filter_status.b.internal_gnss_enabled;
        systemStateMsg.filter_status.DualAntennaHeadingActive = (bool)system_state_packet.filter_status.b.dual_antenna_heading_active;
        systemStateMsg.filter_status.VelocityHeadingEnabled = 	   (bool)system_state_packet.filter_status.b.velocity_heading_enabled;
        systemStateMsg.filter_status.AtmosphericAltitudeEnabled = (bool)system_state_packet.filter_status.b.atmospheric_altitude_enabled;
        systemStateMsg.filter_status.ExternalPositionActive = (bool)system_state_packet.filter_status.b.external_position_active;
        systemStateMsg.filter_status.ExternalVelocityActive = (bool)system_state_packet.filter_status.b.external_velocity_active;
        systemStateMsg.filter_status.ExternalHeadingActive =  (bool)system_state_packet.filter_status.b.external_heading_active;
    systemStateMsg.unix_time_seconds = system_state_packet.unix_time_seconds;
    systemStateMsg.microseconds = system_stapopte_packet.microseconds;
    systemStateMsg.nav_sat_fix.latitude =  system_state_packet.latitude * RADIANS_TO_DEGREES;
    systemStateMsg.nav_sat_fix.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
    systemStateMsg.nav_sat_fix.height =    system_state_packet.height;
    systemStateMsg.velocity_ned.velocity_north = system_state_packet.velocity[0];
    systemStateMsg.velocity_ned.velocity_east =  system_state_packet.velocity[1];
    systemStateMsg.velocity_ned.velocity_down =  system_state_packet.velocity[2];
    systemStateMsg.body_acceleration.accel_north = system_state_packet.body_acceleration[0];
    systemStateMsg.body_acceleration.accel_east =  system_state_packet.body_acceleration[1];
    systemStateMsg.body_acceleration.accel_down =  system_state_packet.body_acceleration[2];
    systemStateMsg.gforce = system_state_packet.g_force;
    systemStateMsg.orientation_rad.roll =    system_state_packet.orientation[0];
    systemStateMsg.orientation_rad.pitch =   system_state_packet.orientation[1];
    systemStateMsg.orientation_rad.heading = system_state_packet.orientation[2];
    systemStateMsg.angular_velocity_rads.x = system_state_packet.angular_velocity[0];
    systemStateMsg.angular_velocity_rads.y = system_state_packet.angular_velocity[1];
    systemStateMsg.angular_velocity_rads.z = system_state_packet.angular_velocity[2];
    systemStateMsg.sat_fix_std_dev.latitude = system_state_packet.standard_deviation[0];
    systemStateMsg.sat_fix_std_dev.longitude = system_state_packet.standard_deviation[1];
    systemStateMsg.sat_fix_std_dev.height =   system_state_packet.standard_deviation[2];
    */

    systemStateMsg.nav_sat_fix.longitude = static_cast<float>( navSatFixMsg.longitude );
    systemStateMsg.nav_sat_fix.latitude = static_cast<float>( navSatFixMsg.latitude );
    systemStateMsg.nav_sat_fix.height =  static_cast<float>( navSatFixMsg.altitude );
    systemStateMsg.velocity_ned.velocity_north = firstPixiVelNED.n;
    systemStateMsg.velocity_ned.velocity_east =  firstPixiVelNED.e;
    systemStateMsg.velocity_ned.velocity_down =  firstPixiVelNED.d;
    //    Value Description
    //    0 Invalid
    //    1 Single Point Position (SPP)
    //    2 Differential GNSS (DGNSS)
    //    3 Float RTK
    //    4 Fixed RTK
    //    5 Dead Reckoning
    //    6 SBAS Position
    switch (fixStatus)
    {
    case 0 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX; break;
    case 1 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_2D_GNSS_FIX; break;
    case 2 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_3D_GNSS_FIX; break;
    case 3 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_RTK_FLOAT_GNSS_FIX; break;
    case 4 : systemStateMsg.filter_status.GNSSFixStatus = fixAsOmniStar ? gps_msgs::FilterStatus::FIX_OMNISTAR_GNSS_FIX : gps_msgs::FilterStatus::FIX_FTK_FIXED_GNSS_FIX; break;
    case 5 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX; break;
    case 6 : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_SBAS_GNSS_FIX; break;
    default : systemStateMsg.filter_status.GNSSFixStatus = gps_msgs::FilterStatus::FIX_NO_GNSS_FIX;
    };

    // Move to radians
    double frontLatRad = deg2Rad(frontPos.lat);
    double frontLonRad = deg2Rad(frontPos.lon);
    double tailLatRad = deg2Rad(tailPos.lat);
    double tailLonRad = deg2Rad(tailPos.lon);

    double y = std::sin(frontLonRad-tailLonRad) * std::cos(frontLatRad);
    double x = std::cos(tailLatRad)*std::sin(frontLatRad) - std::sin(tailLatRad)*std::cos(frontLatRad)*std::cos(frontLonRad-tailLonRad);
    double brng = std::fmod(std::atan2(y, x) + headingOffset + 2.0 * M_PI , 2.0*M_PI);

    systemStateMsg.orientation_rad.heading = brng;

    pub_SystemState.publish(systemStateMsg);

    UTMCoords utmConvert(navSatFixMsg);
    gps_msgs::UTMPosition utmMsg;
    utmMsg.header.stamp = ros::Time::now();
    utmMsg.utm_easting = utmConvert.getX();
    utmMsg.utm_northing = utmConvert.getY();
    utmMsg.utm_height = utmConvert.getZ();
    utmMsg.utm_zone_number = utmConvert.getZone();
    utmMsg.utm_zone_character = utmConvert.getCentralMeridian();

    pub_UTMPosition.publish(utmMsg);
}
