#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <memory>
#include "roscco/EnableDisable.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/ThrottleCmd.h"

class KiaJoyController
{
public:
    KiaJoyController();

private:
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void timerCallback(const ros::TimerEvent& event);
    double linearTranformation(double VALUE, double HIGH_1, double LOW_1, double HIGH_2, double LOW_2);

    ros::NodeHandle _node;
    ros::Publisher _pubEnable;
    ros::Publisher _pubDbwSteering;
    ros::Publisher _pubDbwBrake;
    ros::Publisher _pubDbwThrottle;
    ros::Subscriber _subJoy;
    ros::Subscriber _subRobot;
    ros::Timer _timer;

    int _previous_start_state = 0;
    int _previous_back_state = 0;

    // Number of messages to retain when the message queue is full
    const unsigned int _QUEUE_SIZE = 10;

    // Timed callback frequency set to OSCC recommended publishing rate of 20 Hz (50 ms == 0.05 s)
    const double _CALLBACK_FREQ = 0.05;  // Units in Seconds

    // OSCC input range
    const double _brakeMAX = 1;
    const double _brakeMIN = 0;
    const double _throttleMAX = 1;
    const double _throttleMIN = 0;
    const double _steeringMAX = 1;
    const double _steeringMIN = -1;

    // Store last known value for timed callback
    double _brake = 0.0;
    double _throttle = 0.0;
    double _steering = 0.0;
    double _steeringSupport = 0.0;
    bool _enabled = false;

    const std::size_t _brakeAXES = 5;
    const std::size_t _throttleAXES = 4;
    const std::size_t _steeringAXES = 0;
    const std::size_t _steeringSupportAXES = 2;
    const std::size_t _START_BUTTON = 7;
    const std::size_t _STOP_BUTTON = 6;

    const double _TRIGGER_MIN = 0;
    const double _TRIGGER_MAX = -1;
    const double _JOYSTICK_MIN = -1;
    const double _JOYSTICK_MAX = 1;


};

KiaJoyController::KiaJoyController()
{
    // Timed callback to ensure publishing to OSCC < 200 ms
    _timer = _node.createTimer(ros::Duration(_CALLBACK_FREQ), &KiaJoyController::timerCallback, this);

    _pubEnable = _node.advertise<roscco::EnableDisable>("oscc/enable_disable", _QUEUE_SIZE);
    _pubDbwSteering = _node.advertise<dbw_mkz_msgs::SteeringCmd>("steering_cmd", _QUEUE_SIZE);
    _pubDbwBrake = _node.advertise<dbw_mkz_msgs::BrakeCmd>("brake_cmd", _QUEUE_SIZE);
    _pubDbwThrottle = _node.advertise<dbw_mkz_msgs::ThrottleCmd>("throttle_cmd", _QUEUE_SIZE);
    _subJoy = _node.subscribe<sensor_msgs::Joy>("joy", _QUEUE_SIZE, &KiaJoyController::joystickCallback, this);
}

void KiaJoyController::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    // Map the trigger values [1, -1] to oscc values [0, 1]
    double brakeRepaired = joy->axes[_brakeAXES];
    if (brakeRepaired > 0) brakeRepaired = 0;
    _brake = - brakeRepaired;
    double throttleRepaired = joy->axes[_throttleAXES];
    if (throttleRepaired > 0) throttleRepaired = 0;
    _throttle = -throttleRepaired;
    //_brake = linearTranformation(brakeRepaired, _TRIGGER_MAX, _TRIGGER_MIN, _brakeMAX, _brakeMIN);
    ROS_INFO("joystickCallback %f %f", joy->axes[_brakeAXES], _brake);
    //throttle = linearTranformation(throttleRepaired, _TRIGGER_MAX, _TRIGGER_MIN, _throttleMAX, _throttleMIN);
    _steering = linearTranformation(joy->axes[_steeringAXES], _JOYSTICK_MAX, _JOYSTICK_MIN, _steeringMAX, _steeringMIN);
    _steeringSupport = linearTranformation(joy->axes[_steeringSupportAXES], _JOYSTICK_MAX, _JOYSTICK_MIN, _steeringMAX, _steeringMIN);
    _steering = (_steering + _steeringSupport) / 2.0;
    
    ROS_INFO("joy->buttons[_STOP_BUTTON] --------------------- %d", joy->buttons[_STOP_BUTTON]);
    ROS_INFO("joy->buttons[_START_BUTTON] --------------------- %d", joy->buttons[_START_BUTTON]);
    if ((_previous_back_state == 0) && joy->buttons[_STOP_BUTTON])
    {
        roscco::EnableDisable enable_msg;
        enable_msg.header.stamp = ros::Time::now();
        enable_msg.enable_control = false;
        _pubEnable.publish(enable_msg);
        _enabled = false;
        ROS_INFO("System disabled");
    }
    else if ((_previous_start_state == 0) && joy->buttons[_START_BUTTON])
    {
        roscco::EnableDisable enable_msg;
        enable_msg.header.stamp = ros::Time::now();
        enable_msg.enable_control = true;
        _pubEnable.publish(enable_msg);
        _enabled = true;
        ROS_INFO("System enabled");
    }

    _previous_back_state = joy->buttons[_STOP_BUTTON];
    _previous_start_state = joy->buttons[_START_BUTTON];
}

void KiaJoyController::timerCallback(const ros::TimerEvent& event)
{
    if (_enabled)
    {
        dbw_mkz_msgs::SteeringCmd msgSteeringCmd;
        msgSteeringCmd.enable = true;
        msgSteeringCmd.steering_wheel_angle_cmd = _steering;
        _pubDbwSteering.publish(msgSteeringCmd);

        dbw_mkz_msgs::BrakeCmd msgBrakeCmd;
        msgBrakeCmd.enable = true;
        msgBrakeCmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
        msgBrakeCmd.pedal_cmd = _brake;
        _pubDbwBrake.publish(msgBrakeCmd);

        dbw_mkz_msgs::ThrottleCmd msgThrottleCmd;
        msgThrottleCmd.enable = true;
        msgThrottleCmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
        msgThrottleCmd.pedal_cmd = _throttle;
        _pubDbwThrottle.publish(msgThrottleCmd);
    }
}

double KiaJoyController::linearTranformation(const double VALUE, const double HIGH_1, const double LOW_1, const double HIGH_2,
                                             const double LOW_2)
{
    return LOW_2 + (VALUE - LOW_1) * (HIGH_2 - LOW_2) / (HIGH_1 - LOW_1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kia_joy");

    KiaJoyController controller;

    ros::spin();
}
