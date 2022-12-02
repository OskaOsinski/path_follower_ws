#ifndef KIADATASPEEDCONTROLLER_H
#define KIADATASPEEDCONTROLLER_H

#include <ros/ros.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/GearCmd.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/TurnSignalCmd.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <memory>

extern "C" {
#include <oscc.h>
}


class KiaDataspeedController
{
public:
    KiaDataspeedController(ros::NodeHandlePtr publicNode, ros::NodeHandlePtr privateNode);
    ~KiaDataspeedController();

private:
    void timerCallback(const ros::TimerEvent& event);
    void osccEnableCmdCallback(const std_msgs::Bool::ConstPtr& cmd);
    void dbwSteeringCmdCallback(const dbw_mkz_msgs::SteeringCmd::ConstPtr& cmd);
    void dbwBrakeCmdCallback(const dbw_mkz_msgs::BrakeCmd::ConstPtr& cmd);
    void dbwThrottleCmdCallback(const dbw_mkz_msgs::ThrottleCmd::ConstPtr& cmd);
    void dbwGearCmdCallback(const dbw_mkz_msgs::GearCmd::ConstPtr& cmd);

    ros::NodeHandlePtr _publicNode;
    ros::NodeHandlePtr _privateNode;
    ros::Timer _timer;

    ros::Subscriber _subOsccEnableCmd;
    ros::Subscriber _subDbwSteeringCmd;
    ros::Subscriber _subDbwBrakeCmd;
    ros::Subscriber _subDbwThrottleCmd;
    ros::Subscriber _subDbwGearCmd;

    ros::Publisher _pubDbwSteeringReport;
    ros::Publisher _pubDbwBreakReport;
    ros::Publisher _pubDbwWheelSpeedReport;
    ros::Publisher _pubDbwThrottleReport;
    ros::Publisher _pubDbwEnableDbw;
    ros::Publisher _pubDbwGearReport;
    ros::Publisher _pubButtonStatus;
    ros::Publisher _pubVehicleTwist;
    
    bool _enableOscc = false;

    // Number of messages to retain when the message queue is full
    const unsigned int _QUEUE_SIZE = 10;

    // Timed callback frequency set to OSCC recommended publishing rate of 20 Hz (50 ms == 0.05 s)
    const double _CALLBACK_FREQ = 0.01;  // Units in Seconds


    // Store last known value for timed callback

    double _throttleCmd = 0.0;
    double _breakCmd = 0.0;

    int _gearSetPoint = 0;
    bool _waitForGearChange = false;
    bool _dbwEnabledSend = false;

    double _actualSpeed = 0.0;

    int _osccCanChannel = 0;
    int _chassiCanChannel = 1;

    double actualSteeringSetPoint = 0.0;
    double previousSteeringSetpoint = 0.0;
    double rampSteeringSetPoint = 0.0;
    double STEERING_WHEEL_SPEED = 400; // degree/ second

};


#endif // KIADATASPEEDCONTROLLER_H
