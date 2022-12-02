#ifndef smart_vehicle
#define smart_vehicle_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include<queue>
#include<geometry_msgs/PoseStamped.h>
#include <chrono>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>

#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/GearCmd.h>
#include <stdint.h>

        static float PATH_LEN_RADIUS = 0.0;
        static float UPPER_RADIUS_VAL = 0.0;
        static float MIN_TIME_TO_DEST_POINT = 0.0;
        static float LOCAL_PATH_MAX_LEN = 0.0;
        static float CAR_WIDTH = 0.0;

        static float MAX_VEL_DRIVE = 0.0;
        static float MIN_VEL_DRIVE = 0.0;
        static float AVG_VEL_DRIVE = 0.0;
        static float HEADING_TIME_CONS = 0.0;
        
        static float MAX_THROTTLE_PERCENT = 0.0;
        static float KP_THROTTLE = 0.0;
	    static float KP_BRAKE = 0.0;
        static float KP_HEADING = 0.0;
        static float KP_DIST = 0.0;
        static float STOP_CAR_HIST = 1.5;
        static float MAX_BRAKE_PERCENT = 0.0;
        
        static float RAMP_VEL_PER_SEC = 1.0;
        static float VEL_HIST_ON_GEAR = 1.5;

class SmartVehicleSteeringClass
{
    public:
        SmartVehicleSteeringClass(
            ros::NodeHandle nh);
            
    private:
        geometry_msgs::Point carPos_;
        geometry_msgs::Point prevTailPoint_;
        geometry_msgs::Point leftBWPos_;
        geometry_msgs::Point rightBWPos_;
        std_msgs::Float64 velocityCmd_;
        std_msgs::Float64 steeringAngleCmd_;
        std_msgs::Bool brakeCmd_;
        bool writeDebugToConsole = false;
        bool b_emBrake_ = false;
        bool b_gotBrakeMsg_ = false;
        bool b_gotPathMsg_ = false;
        bool b_gotHeadingMsg_ = false;
        bool b_allowDriving_ = true;
        bool b_destReached_ = false;
        int lPathCutIndex_ = 0;
        int currentState_ = 0;
        int previousState_ = 0;
        int currentThrottleState_ = 0;
        float lineHeading_ = 0.0;
        bool b_lineOnLeft_ = false;
        bool b_readyToPrepare_ = false;
        float curveRadius_ = 0.0;
        float throttlePercent_ = 0.0;
        bool b_curveAhead = false;
        float brakePercent_ = 0.0;
        float rampingVelocity_ = 0.0;
        std::chrono::high_resolution_clock::time_point startTime_;
        float timeCycle_ = 0.0;
        float pathLenToDest_ = 0.0;

        dbw_mkz_msgs::GearCmd gearState_;

        std_msgs::Bool b_newPathRequest_;

        int numRemovedPt_ = 0;

        bool b_pathCrossed = false;
        bool b_pathValid_ = false;

        
        float lPathCurrentLen_ = 0.0;
        float carHeading_ = 0.0;
        float headingErrorToPID_ = 0.0;
        float distErrorToPID_ = 0.0;
        float actualCarSpeed_ = 0.0;

		bool b_pathCrossed_ = false;

        std::vector<std::pair <geometry_msgs::Point, float> > globalPath_;
        std::vector<std::pair <geometry_msgs::Point, float> > pathToFollow_;
        std::vector<std::pair <geometry_msgs::Point, float> > localPath_;
        std::pair <geometry_msgs::Point, float> lastDelLocalPoint_;
        
        ros::Publisher pubNewPathRequest;
        ros::Publisher pubDbwBrake;
        ros::Publisher pubDbwThrottle;
        ros::Publisher pubDbwSteeringAngle;
        ros::Publisher pubPathToFollow;
        
        ros::Subscriber subDbwGearReport;

        ros::Publisher pubDebugToRqt;
        ros::Subscriber subValidPath;
	ros::Publisher pubLocalPath;
   
        ros::Subscriber subDbwSteeringReport;
        dbw_mkz_msgs::SteeringReport actualReport;
        
        ros::Publisher pubVelocity;
        ros::Publisher pubAskForGearChange;
        ros::Publisher pubSteeringAngle;
        ros::Subscriber subEmergencyBrake;
        ros::Subscriber subStopButton;
        ros::Subscriber subPath;
        ros::Subscriber subCarPos;
        ros::Subscriber subCarHeading;


        void mainCallback(
            const geometry_msgs::PoseStamped::ConstPtr& msg);

        void initAdvertisers(
            ros::NodeHandle& nh);

        void validPathCallback(
            const std_msgs::Bool::ConstPtr& msg);

        void gearStateCallback(
            const dbw_mkz_msgs::GearReport::ConstPtr& msg);

        void pathCallback(
            const nav_msgs::Path::ConstPtr& msg);

        void emBrakeCallback(
            const std_msgs::Bool::ConstPtr& msg);

        void carHeadingCallback(
            const std_msgs::Float64::ConstPtr& msg);

        void carSteeringReportCallback(
            const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);

        float getDist(
            geometry_msgs::Point p1,
            geometry_msgs::Point p2);

        float getEnuHeading(
            geometry_msgs::Point tailPoint,
            geometry_msgs::Point headPoint);

        float getDistToLine(
            geometry_msgs::Point point, 
            geometry_msgs::Point lineP1,
            geometry_msgs::Point lineP2);

        bool arePointsTheSame(
            geometry_msgs::Point p1, 
            geometry_msgs::Point p2);

        void finiteStateMachine();

        void estimateVelocity();

        void estimateErrors();

        void estimateSteeringAngle();

        void publishCmds();

        void updateLocalPath();

        void calculateBackWheelsPositions();

        bool isBehindCar(
            geometry_msgs::Point point);

        void printDebLog();

        void publishDebugToVis();

        bool isPathCrossed();

        bool isDestReached();

        void estimateThrottlePercent();

        void rampingVelocity();

};
#endif  // rqt_oscc_cmd_MY_PLUGIN_H

