#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include<geometry_msgs/Point.h>

#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>

#include <dbw_mkz_msgs/SteeringReport.h>

static float LIDAR_PITCH_ANGLE;
static float LIDAR_HEIGHT_POSITION;
static bool FRONT_LIDAR_FLAG;

class SmartVehicleSimulatorClass
{
    public:
        SmartVehicleSimulatorClass(
            ros::NodeHandle nh);
    private:
        float carVelocity_ = 0.0;
        std_msgs::Bool emBrakeDataCmd_;
        bool brakeData_ = false;
        float steeringAngle_ = 0.0;
        geometry_msgs::Point carPos_;
        std_msgs::Float64 carHeadingCmd_;
        bool b_gotVeloMsg_ = false;
        bool b_gotBrakeMsg_ = false;
        float throttlePercent_ = 0.0;
        float realVelocity_ = 0.0;
        
        const float SIM_RATE = 30.0;
        ros::Timer timer_;

        ros::Publisher pubActCarVel;
        ros::Publisher pubCarPos;
        ros::Publisher pubEmergencyBrake;
        ros::Publisher pubSimHeading;
        ros::Subscriber subSteeringAngle;
        ros::Subscriber subVelocity;
        ros::Subscriber subBrake;
        ros::Subscriber subThrottle;

        void timerCallback(const ros::TimerEvent &event);


        void throttleCallback(const dbw_mkz_msgs::ThrottleCmd &msg);

        float getEnuHeading(
            geometry_msgs::Point tailPoint,
            geometry_msgs::Point headPoint);

        void steeringAngleCallback(
            const std_msgs::Float64::ConstPtr& msg);

        void velocityCallback(
            const std_msgs::Float64::ConstPtr& msg);

        void brakeCallback(
            const std_msgs::Bool::ConstPtr& msg);

        void drive();

        void publishCmds();










    };

