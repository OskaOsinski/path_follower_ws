#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <SmartVehicleSimulatorClass.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseStamped.h>
#include <thread>
#include <math.h>
#include <chrono>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


SmartVehicleSimulatorClass::SmartVehicleSimulatorClass(
    ros::NodeHandle nh) 
{
    subSteeringAngle = nh.subscribe ("steering_angle", 0, &SmartVehicleSimulatorClass::steeringAngleCallback, this);
    subVelocity = nh.subscribe ("steering_velocity", 0, &SmartVehicleSimulatorClass::velocityCallback, this);
    subThrottle = nh.subscribe ("/vehicle/throttle_cmd", 0, &SmartVehicleSimulatorClass::throttleCallback, this);
    subBrake = nh.subscribe ("/steering_brake", 0, &SmartVehicleSimulatorClass::brakeCallback, this);
    pubCarPos = nh.advertise<geometry_msgs::PoseStamped> ("vehicle_pose", 0);
    pubEmergencyBrake = nh.advertise<std_msgs::Bool> ("/sim_emergency_brake", 0);
    pubSimHeading = nh.advertise<std_msgs::Float64> ("vehicle_enu_heading", 0);

    pubActCarVel = nh.advertise<dbw_mkz_msgs::SteeringReport> ("/vehicle/steering_report", 0);


    double rate = SIM_RATE;
    timer_ = nh.createTimer(ros::Duration(1.0 / rate), &SmartVehicleSimulatorClass::timerCallback, this);

    carPos_.x = 0.0;
    carPos_.y = 0.0;
    emBrakeDataCmd_.data = false;
    carHeadingCmd_.data = 0.0;


    //nh.getParam("steering_nodelet/yawLidar", LIDAR_YAW_ANGLE);
   // nh.getParam("steering_nodelet/frontLidarFlag", FRONT_LIDAR_FLAG);
    //nh.getParam("steering_nodelet/heightPositionLidar", LIDAR_HEIGHT_POSITION);
    //nh.getParam("steering_nodelet/latPositionLidar", LIDAR_LAT_DEVIATION_POSITION);
} 


void SmartVehicleSimulatorClass::throttleCallback(const dbw_mkz_msgs::ThrottleCmd& msg)
{
    throttlePercent_ = msg.pedal_cmd;
}


void SmartVehicleSimulatorClass::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
    publishCmds();
    ROS_INFO("---------------------------------");
    ROS_INFO("carPos_: %f - %f ", carPos_.x, carPos_.y);
    ROS_INFO("carHeadingCmd_: %f ", carHeadingCmd_.data * 180/M_PI);
    ROS_INFO("emBrakeDataCmd_: %d ", emBrakeDataCmd_.data);
}


void SmartVehicleSimulatorClass::steeringAngleCallback(
    const std_msgs::Float64::ConstPtr& msg)
{
   steeringAngle_ = msg->data;
   drive();
}

void SmartVehicleSimulatorClass::velocityCallback(
    const std_msgs::Float64::ConstPtr& msg)
{
   carVelocity_ = msg->data;
   b_gotVeloMsg_ = true;
}

void SmartVehicleSimulatorClass::brakeCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
   brakeData_ = msg->data;
   b_gotBrakeMsg_ = true;
}


float SmartVehicleSimulatorClass::getEnuHeading(
    geometry_msgs::Point tailPoint,
    geometry_msgs::Point headPoint)
{
    fmod(atan2(headPoint.y - tailPoint.y, headPoint.x - tailPoint.x)+2*M_PI, 2*M_PI);
}


void SmartVehicleSimulatorClass::drive()
{
    float d_x = 0.0;
    float d_y = 0.0;
    float drivenDist = 0.0;
    
    //pseudo car response
    carHeadingCmd_.data += steeringAngle_ / SIM_RATE ;
    carHeadingCmd_.data = fmod( carHeadingCmd_.data+2*M_PI, 2*M_PI);
    //
    realVelocity_ += throttlePercent_ * 1.0;
    realVelocity_ -= brakeData_ * 0.5;
    if(realVelocity_ < 0.0)
    {
        realVelocity_ = 0.0;
    }
    //
    if( b_gotVeloMsg_ && b_gotBrakeMsg_ )
    {
        drivenDist = realVelocity_ / SIM_RATE;
        d_x = drivenDist * cos(carHeadingCmd_.data);
        d_y = drivenDist * sin(carHeadingCmd_.data);
        carPos_.x += d_x;
        carPos_.y += d_y;
    }
}


void SmartVehicleSimulatorClass::publishCmds()
{
    geometry_msgs::PoseStamped stampedCarPos;
    stampedCarPos.header.frame_id = "map";
    stampedCarPos.pose.position = carPos_;
    pubCarPos.publish(stampedCarPos);
    emBrakeDataCmd_.data = false;
    pubEmergencyBrake.publish(emBrakeDataCmd_);
    pubSimHeading.publish(carHeadingCmd_);
    dbw_mkz_msgs::SteeringReport stReport;
    stReport.speed = realVelocity_;
    pubActCarVel.publish(stReport);
}

