#include "kia_dataspeed_controller.h"

#include "vehicles/kia_niro.h"
#include "geometry_msgs/TwistStamped.h"

oscc_brake_report_s gloablActualOsccBrakeReport;
oscc_steering_report_s gloablActualOsccSteeringReport;
oscc_throttle_report_s gloablActualOsccThrottleReport;
oscc_fault_report_s gloablActualOsccFaultReport;

std::uint8_t gloablGearRawValue = 0;
double globalActualSteeringWheelAngle = 0.0;
kia_soul_obd_wheel_speed_data_s globalActualWheelSpeedReport;
double globalActualBreakPressure = 0.0;
bool globalActualBrakeLightsActive = 0.0;
double globalActualBrakePosition = 0.0;
double globalActualThrottlePosition = 0.0;
double globalActualCarSpeedKph = 0.0;
double globalYawRate = 0.0;

double map(double val, double min1, double max1, double min2, double max2)
{
    return (val-min1)*(max2-min2)/(max1-min1) + min2;
}

void brake_callback(oscc_brake_report_s *report)
{
    //ROS_INFO(("Call function %s", __PRETTY_FUNCTION__ ));
    gloablActualOsccBrakeReport = *report;
}

void steering_callback(oscc_steering_report_s *report)
{
    //ROS_INFO(("Call function %s", __PRETTY_FUNCTION__ ));
    gloablActualOsccSteeringReport = *report;
}

void throttle_callback(oscc_throttle_report_s *report)
{
    //ROS_INFO(("Call function %s", __PRETTY_FUNCTION__ ));
    gloablActualOsccThrottleReport = *report;
}

void fault_callback(oscc_fault_report_s *report)
{
    //ROS_INFO(("Call function %s", __PRETTY_FUNCTION__ ));
    gloablActualOsccFaultReport = *report;
}

void obd_callback(can_frame *canFrame)
{
    //ROS_INFO("Call function %s %d", __PRETTY_FUNCTION__, canFrame->can_id);
    switch(canFrame->can_id)
    {
    case KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID :
        get_steering_wheel_angle(canFrame, &globalActualSteeringWheelAngle);
        break;
    case KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID :
        memcpy(&globalActualWheelSpeedReport, canFrame->data, sizeof(globalActualWheelSpeedReport));
        break;
    case KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID :
        get_brake_pressure(canFrame, &globalActualBreakPressure);

        std::uint16_t yawRate;
        memcpy(&yawRate, canFrame->data + 5, 2);
        globalYawRate = yawRate & 0x1fff;
        globalYawRate = (globalYawRate * 0.01) - 40.95;
        break;
    case KIA_SOUL_OBD_THROTTLE_PRESSURE_CAN_ID :
        std::uint8_t brakePosition;
        memcpy(&brakePosition, canFrame->data, 1);
        globalActualBrakePosition = map(brakePosition, 0.0, 162.0, 0.15, 0.50);
        std::uint8_t throttlePosition;
        memcpy(&throttlePosition, canFrame->data+7, 1);
        globalActualThrottlePosition = map(throttlePosition, 0, 251, 0.15, 0.80);
        break;
    case KIA_SOUL_OBD_SPEED_CAN_ID:
        std::uint8_t carSpeedKph;
        memcpy(&carSpeedKph, canFrame->data+6, 1 );
        globalActualCarSpeedKph = carSpeedKph;
        break;
      
    case 0x596:
        std::int8_t brakeLights;
        memcpy(&brakeLights, canFrame->data+2, 1);
        if(brakeLights == 0x10)
            globalActualBrakeLightsActive = false;
        else if(brakeLights == 0x20)
            globalActualBrakeLightsActive = true;
        break;
    case 0x372 :
        memcpy(&gloablGearRawValue, canFrame->data+2, 1);
        break;
    }
}

KiaDataspeedController::KiaDataspeedController(ros::NodeHandlePtr publicNode, ros::NodeHandlePtr privateNode)
{
    _publicNode = publicNode;
    _privateNode = privateNode;

    oscc_subscribe_to_brake_reports(brake_callback);
    oscc_subscribe_to_steering_reports(steering_callback);
    oscc_subscribe_to_throttle_reports(throttle_callback);
    oscc_subscribe_to_fault_reports(fault_callback);
    oscc_subscribe_to_obd_messages(obd_callback);

    _privateNode->param<int>("oscc_can_channel", _osccCanChannel, 0);
    _privateNode->param<int>("chassi_can_channel", _chassiCanChannel, 1);

    ROS_INFO("Setting oscc can channel: %d, chassi can channel: %d", _osccCanChannel, _chassiCanChannel);

    // Timed callback to ensure publishing to OSCC < 200 ms
    _timer = _publicNode->createTimer(ros::Duration(_CALLBACK_FREQ), &KiaDataspeedController::timerCallback, this);

    _subOsccEnableCmd = _publicNode->subscribe<std_msgs::Bool>("vehicle/oscc_enable_cmd", _QUEUE_SIZE, &KiaDataspeedController::osccEnableCmdCallback, this);
    _subDbwSteeringCmd = _publicNode->subscribe<dbw_mkz_msgs::SteeringCmd>("vehicle/steering_cmd", _QUEUE_SIZE, &KiaDataspeedController::dbwSteeringCmdCallback, this);
    _subDbwBrakeCmd = _publicNode->subscribe<dbw_mkz_msgs::BrakeCmd>("vehicle/brake_cmd", _QUEUE_SIZE, &KiaDataspeedController::dbwBrakeCmdCallback, this);
    _subDbwThrottleCmd = _publicNode->subscribe<dbw_mkz_msgs::ThrottleCmd>("vehicle/throttle_cmd", _QUEUE_SIZE, &KiaDataspeedController::dbwThrottleCmdCallback, this);
    _subDbwGearCmd = _publicNode->subscribe<dbw_mkz_msgs::GearCmd>("vehicle/gear_cmd", _QUEUE_SIZE, &KiaDataspeedController::dbwGearCmdCallback, this);
    
    _pubDbwEnableDbw = _publicNode->advertise<std_msgs::Bool>("vehicle/dbw_enabled", _QUEUE_SIZE);
    _pubDbwSteeringReport = _publicNode->advertise<dbw_mkz_msgs::SteeringReport>("vehicle/steering_report", _QUEUE_SIZE);
    _pubDbwBreakReport = _publicNode->advertise<dbw_mkz_msgs::BrakeReport>("vehicle/brake_report", _QUEUE_SIZE);
    _pubDbwWheelSpeedReport = _publicNode->advertise<dbw_mkz_msgs::WheelSpeedReport>("vehicle/wheel_speed_report", _QUEUE_SIZE);
    _pubDbwThrottleReport = _publicNode->advertise<dbw_mkz_msgs::ThrottleReport>("vehicle/throttle_report", _QUEUE_SIZE);
    _pubDbwGearReport = _publicNode->advertise<dbw_mkz_msgs::GearReport>("vehicle/gear_report", _QUEUE_SIZE);
    _pubVehicleTwist = _publicNode->advertise<geometry_msgs::TwistStamped>("/as_rx/vehicle_motion", _QUEUE_SIZE);

    bool ret = false;
    ret = oscc_open_fevcan_oscc(_osccCanChannel);
    if (ret != OSCC_OK)
    {
        ROS_ERROR("Could not initialize OSCC for can channel %d", _osccCanChannel);
        throw std::runtime_error("Cannot create ossc can channel");
    }

    ret = oscc_open_fevcan_vehicle(_chassiCanChannel);
    if (ret != OSCC_OK)
    {
        ROS_ERROR("Could not initialize chassi for can channel %d", _chassiCanChannel);
        throw std::runtime_error("Cannot create chassi can channel");
    }

}

KiaDataspeedController::~KiaDataspeedController()
{
    int ret = oscc_disable();
    if (ret != OSCC_OK)
    {
        ROS_ERROR("Could not disable OSCC");
    }

    ret = oscc_close(_osccCanChannel);
    if (ret != OSCC_OK)
    {
        ROS_ERROR("Could not close OSCC connection");
    }

    ret = oscc_close(_chassiCanChannel);
    if (ret != OSCC_OK)
    {
        ROS_ERROR("Could not close OSCC connection");
    }

}

void KiaDataspeedController::osccEnableCmdCallback(const std_msgs::Bool::ConstPtr &cmd)
{
    if(cmd->data)
    {
        _enableOscc = true;
        oscc_enable();
        ROS_INFO("oscc_enable()");
    }
    else
    {
        _enableOscc = false;
        oscc_disable();
        ROS_INFO("oscc_disable()");
    }
}

void KiaDataspeedController::dbwSteeringCmdCallback(const dbw_mkz_msgs::SteeringCmd::ConstPtr &cmd)
{
    if (cmd->enable)
    {
        actualSteeringSetPoint = - cmd->steering_wheel_angle_cmd * 180.0 / 3.14;
    }
}

void KiaDataspeedController::dbwBrakeCmdCallback(const dbw_mkz_msgs::BrakeCmd::ConstPtr &cmd)
{
    if (cmd->enable )
    {
        if (cmd->pedal_cmd_type == dbw_mkz_msgs::BrakeCmd::CMD_PERCENT)
        {
            _breakCmd = cmd->pedal_cmd;
            oscc_publish_brake_position(_breakCmd);
        }
        else if (cmd->pedal_cmd_type == dbw_mkz_msgs::BrakeCmd::CMD_TORQUE)
        {
            _breakCmd = double (cmd->pedal_cmd) / double(dbw_mkz_msgs::BrakeCmd::TORQUE_MAX);
            oscc_publish_brake_position(_breakCmd);
        }
    }
}

void KiaDataspeedController::dbwThrottleCmdCallback(const dbw_mkz_msgs::ThrottleCmd::ConstPtr &cmd)
{
    if (cmd->enable )
    {
        if (cmd->pedal_cmd_type == dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT)
        {
            _throttleCmd = cmd->pedal_cmd;
            oscc_publish_throttle_position(_throttleCmd);
        }
        else if (cmd->pedal_cmd_type == dbw_mkz_msgs::ThrottleCmd::CMD_PEDAL)
        {
            _throttleCmd = (cmd->pedal_cmd - 0.15)/(0.80-0.15);
            oscc_publish_throttle_position(_throttleCmd);
        }
        else
        {
            ROS_WARN("KIA oscc support throttle only as percent value or unit less value");
        }
    }
}

void KiaDataspeedController::dbwGearCmdCallback(const dbw_mkz_msgs::GearCmd::ConstPtr& cmd)
{
    _gearSetPoint = cmd->cmd.gear;
}


void KiaDataspeedController::timerCallback(const ros::TimerEvent& event)
{
    double deltaTime = 0.05;
    // ROS_ERROR("OSCC _enableOscc: %d", _enableOscc );
    if(_enableOscc) {
        oscc_enable();
    }
    if (gloablActualOsccSteeringReport.enabled)
    {
        if(std::abs(actualSteeringSetPoint - previousSteeringSetpoint) < STEERING_WHEEL_SPEED * deltaTime) //RAMP is not needed
        {
            oscc_publish_steering_angle(actualSteeringSetPoint, STEERING_WHEEL_SPEED * 10);
            //ROS_INFO("Print steering_cmd.angle: %d", steering_cmd.angle);
            previousSteeringSetpoint = actualSteeringSetPoint;
            rampSteeringSetPoint = actualSteeringSetPoint;

        }else{
            if(actualSteeringSetPoint < 0)
            {
                if(actualSteeringSetPoint < previousSteeringSetpoint)
                {
                    //ROS_INFO("actualSteeringSetPoint: %f", actualSteeringSetPoint);
                    //ROS_INFO("previousSteeringSetpoint: %f", previousSteeringSetpoint);
                    rampSteeringSetPoint = previousSteeringSetpoint - deltaTime * STEERING_WHEEL_SPEED;
                    oscc_publish_steering_angle(rampSteeringSetPoint, STEERING_WHEEL_SPEED * 10 );
                    //ROS_INFO("steering_cmd.angle: %d", steering_cmd.angle);
                    //ROS_INFO("deltaTime: %f", deltaTime);
                }else{
                    rampSteeringSetPoint = previousSteeringSetpoint + deltaTime * STEERING_WHEEL_SPEED;
                    oscc_publish_steering_angle(rampSteeringSetPoint, STEERING_WHEEL_SPEED * 10);
                }
            }else{ 
                if(actualSteeringSetPoint > previousSteeringSetpoint)
                {
                    rampSteeringSetPoint = previousSteeringSetpoint + deltaTime * STEERING_WHEEL_SPEED;
                    oscc_publish_steering_angle(rampSteeringSetPoint, STEERING_WHEEL_SPEED * 10);
                    //ROS_INFO("steering_cmd.angle: %d", steering_cmd.angle);
                }else{
                    rampSteeringSetPoint = previousSteeringSetpoint - deltaTime * STEERING_WHEEL_SPEED;
                    oscc_publish_steering_angle(rampSteeringSetPoint, STEERING_WHEEL_SPEED * 10);
                    //ROS_INFO("steering_cmd.angle: %d", steering_cmd.angle);
                }
            }
            previousSteeringSetpoint = rampSteeringSetPoint;
        }
    }

    std_msgs::Bool enMsg;
    enMsg.data = gloablActualOsccBrakeReport.enabled && gloablActualOsccSteeringReport.enabled && gloablActualOsccThrottleReport.enabled;
    // enMsg.data  = true;
    _pubDbwEnableDbw.publish(enMsg);

    dbw_mkz_msgs::Gear gearMsg;
    dbw_mkz_msgs::GearReport gearReport;
    gearReport.header.stamp = ros::Time::now();
//    ROS_INFO("globalGearRawValue %d", gloablGearRawValue);
    
    switch(gloablGearRawValue)
    {
        
    case 0 : case 16 : case 32 : case 48 : 
        gearMsg.gear = 1;  break; //PARK
    case 5 : case 21 : case 37 : case 53 :
        gearMsg.gear = 4; break; //DRIVE
    case 6 : case 22 : case 38 : case 54 :
        gearMsg.gear = 3; break; //DRIVE
    case 7 : case 23 : case 39 : case 55 :
        gearMsg.gear = 2; break; //REVERSE
    default : gearMsg.gear = 4;
    }
    gearReport.state = gearMsg;
    gearReport.cmd = gearMsg;
    gearReport.reject.value = dbw_mkz_msgs::GearReject::NONE;
    gearReport.override = false;
    gearReport.fault_bus = false;
    _pubDbwGearReport.publish(gearReport);

    _waitForGearChange = _gearSetPoint != gearMsg.gear;

    dbw_mkz_msgs::SteeringReport steeringReport;
    steeringReport.header.stamp = ros::Time::now();
    steeringReport.enabled = gloablActualOsccSteeringReport.enabled;
    steeringReport.override = gloablActualOsccSteeringReport.operator_override;
    steeringReport.steering_wheel_angle = - globalActualSteeringWheelAngle * 3.14/180.0;
    steeringReport.steering_wheel_cmd = - rampSteeringSetPoint * 3.14/180.0 ;
    steeringReport.steering_wheel_torque = 0.0;
    steeringReport.speed = globalActualCarSpeedKph * 10.0/36.0;
    steeringReport.timeout = false;
    steeringReport.fault_wdc = false;
    steeringReport.fault_bus1 = false;
    steeringReport.fault_bus2 = false;
    steeringReport.fault_calibration = false;
    steeringReport.fault_power = false;
    _pubDbwSteeringReport.publish(steeringReport);

    dbw_mkz_msgs::BrakeReport breakeReport;
    breakeReport.header.stamp = ros::Time::now();
    breakeReport.enabled = gloablActualOsccBrakeReport.enabled;
    breakeReport.override = gloablActualOsccBrakeReport.operator_override;
    breakeReport.pedal_input = 0.0;
    breakeReport.pedal_cmd = map(_breakCmd, 0.0, 1.0, 0.15, 0.50);
    breakeReport.pedal_output = globalActualBrakePosition;
    breakeReport.torque_input = 0.0;
    breakeReport.torque_cmd = 0.0;
    breakeReport.torque_output = 0.0;
    // breakeReport.boo_input = globalActualBrakeLightsActive;
    breakeReport.boo_cmd = globalActualBrakeLightsActive;
    // breakeReport.boo_output = globalActualBrakeLightsActive;
    breakeReport.driver = false;
    breakeReport.timeout = false;
    breakeReport.watchdog_counter.source = dbw_mkz_msgs::WatchdogCounter::NONE;
    breakeReport.watchdog_braking = false;
    breakeReport.fault_wdc = false;
    breakeReport.fault_ch1 = false;
    breakeReport.fault_ch2 = false;
    breakeReport.fault_power = false;
    _pubDbwBreakReport.publish(breakeReport);
    // ROS_ERROR ("Brake enabled: %d, and operator_override: %d", breakeReport.enabled, breakeReport.override );

    dbw_mkz_msgs::ThrottleReport throttleReport;
    throttleReport.header.stamp = ros::Time::now();
    throttleReport.enabled = gloablActualOsccThrottleReport.enabled;
    throttleReport.override = gloablActualOsccThrottleReport.operator_override;
    throttleReport.pedal_input = 0.0;
    throttleReport.pedal_cmd = map(_throttleCmd, 0.0, 1.0, 0.15, 0.80);
    throttleReport.pedal_output = globalActualThrottlePosition;
    throttleReport.driver = false;
    throttleReport.timeout = false;
    throttleReport.watchdog_counter.source = dbw_mkz_msgs::WatchdogCounter::NONE;
    throttleReport.fault_wdc = false;
    throttleReport.fault_ch1 = false;
    throttleReport.fault_ch2 = false;
    throttleReport.fault_power = false;
    _pubDbwThrottleReport.publish(throttleReport);

    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = globalActualCarSpeedKph;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;

    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = globalYawRate * 0.0174532925;
    _pubVehicleTwist.publish(twist);

    if(throttleReport.override || breakeReport.override || steeringReport.override) {
        _enableOscc = false;
    }

}


