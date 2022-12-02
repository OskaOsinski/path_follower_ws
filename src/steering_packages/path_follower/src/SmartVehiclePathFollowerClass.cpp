#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <SmartVehiclePathFollowerClass.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <thread>
#include <math.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <queue>
#include <geometry_msgs/PoseStamped.h>
#include <path_follower_msgs/PathFollowerDebugInfo.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/GearCmd.h>


SmartVehicleSteeringClass::SmartVehicleSteeringClass(
    ros::NodeHandle nh) 
{
    subCarPos = nh.subscribe ("vehicle_pose", 0, &SmartVehicleSteeringClass::mainCallback, this);
    subCarHeading = nh.subscribe ("vehicle_enu_heading", 0, &SmartVehicleSteeringClass::carHeadingCallback, this);
    subPath = nh.subscribe ("path_to_follow", 0, &SmartVehicleSteeringClass::pathCallback, this);
    subEmergencyBrake = nh.subscribe ("emergency_brake", 0, &SmartVehicleSteeringClass::emBrakeCallback, this);
    subEmergencyBrake = nh.subscribe ("/stop_button_from_rqt", 0, &SmartVehicleSteeringClass::emBrakeCallback, this);
    subDbwSteeringReport = nh.subscribe ("vehicle/steering_report", 0, &SmartVehicleSteeringClass::carSteeringReportCallback, this);
    subDbwGearReport = nh.subscribe ("vehicle/gear_report", 0, &SmartVehicleSteeringClass::gearStateCallback, this);

    subValidPath = nh.subscribe ("valid_path", 0, &SmartVehicleSteeringClass::validPathCallback, this);


    initAdvertisers(nh);

    nh.getParam("maxThrottlePercent", MAX_THROTTLE_PERCENT);
    nh.getParam("maxBrakePercent", MAX_BRAKE_PERCENT);
    nh.getParam("KP_throttle", KP_THROTTLE);
    nh.getParam("KP_brake", KP_BRAKE);
    nh.getParam("KP_heading", KP_HEADING);
    nh.getParam("KP_dist", KP_DIST);

    nh.getParam("maxVelocity", MAX_VEL_DRIVE);
    nh.getParam("minVelocity", MIN_VEL_DRIVE);
    nh.getParam("casualVelocity", AVG_VEL_DRIVE);
    nh.getParam("headingTimeCons", HEADING_TIME_CONS);

    nh.getParam("pathLenCurveEstim", PATH_LEN_RADIUS);
    nh.getParam("upperRadiusCurve", UPPER_RADIUS_VAL);
    nh.getParam("minTimeToDest", MIN_TIME_TO_DEST_POINT);
    nh.getParam("localPathMaxLen", LOCAL_PATH_MAX_LEN);
    //nh.getParam("/vehicle_data/kia_niro/width", CAR_WIDTH);
    CAR_WIDTH = 5.0;
    nh.getParam("stopCarHist", STOP_CAR_HIST);

    nh.getParam("rampVelPerSec", RAMP_VEL_PER_SEC);
    nh.getParam("slowOnGearHist", VEL_HIST_ON_GEAR);
    
    nh.getParam("print_debug_messages", writeDebugToConsole);

    b_newPathRequest_.data = false;
    startTime_ = std::chrono::high_resolution_clock::now();
} 

void SmartVehicleSteeringClass::initAdvertisers(ros::NodeHandle& nh)
{

    pubLocalPath = nh.advertise<nav_msgs::Path>("local_path", 0);

    pubSteeringAngle = nh.advertise<std_msgs::Float64> ("steering_angle", 0);
    pubVelocity = nh.advertise<std_msgs::Float64> ("steering_velocity", 0);
    pubNewPathRequest = nh.advertise<std_msgs::Bool> ("stering_new_path_request", 0);
    pubDbwBrake = nh.advertise<dbw_mkz_msgs::BrakeCmd> ("vehicle/brake_cmd", 0);;
    pubDbwThrottle = nh.advertise<dbw_mkz_msgs::ThrottleCmd> ("vehicle/throttle_cmd", 0);;
    pubDbwSteeringAngle = nh.advertise<dbw_mkz_msgs::SteeringCmd> ("vehicle/steering_cmd", 0);
    pubPathToFollow = nh.advertise<nav_msgs::Path> ("path_before_car", 0);
    pubAskForGearChange = nh.advertise<dbw_mkz_msgs::GearCmd> ("ask_for_gear_change", 0);

    pubDebugToRqt = nh.advertise<path_follower_msgs::PathFollowerDebugInfo> ("planner_debug_info", 0);
}

void SmartVehicleSteeringClass::mainCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    carPos_.x = msg->pose.position.x;
    carPos_.y = msg->pose.position.y;

    finiteStateMachine();
    if(writeDebugToConsole)
    {
        printDebLog();
    }
    publishDebugToVis();
    publishCmds();


}


void SmartVehicleSteeringClass::validPathCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    b_pathValid_ = msg->data;
}


void SmartVehicleSteeringClass::gearStateCallback(
    const dbw_mkz_msgs::GearReport::ConstPtr& msg)
{
   gearState_.cmd.gear = msg->state.gear;
}


void SmartVehicleSteeringClass::carHeadingCallback(
    const std_msgs::Float64::ConstPtr& msg)
{
    carHeading_ = msg->data;
    b_gotHeadingMsg_ = true;
}

void SmartVehicleSteeringClass::carSteeringReportCallback(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg)
{
    actualReport = *msg;
    actualCarSpeed_ = actualReport.speed;
}


void SmartVehicleSteeringClass::pathCallback(
    const nav_msgs::Path::ConstPtr& msg)
{
    std::pair<geometry_msgs::Point, float> lenPoint;
    geometry_msgs::Point prevPoint;

    if (globalPath_.size() == 0 && msg->poses.size() > 0)
    {

        prevPoint = msg->poses[0].pose.position;
        lenPoint.first = prevPoint;
        lenPoint.second = 0.0;
        globalPath_.push_back(lenPoint);
        pathToFollow_.push_back(lenPoint);
        for (size_t i = 1; i < msg->poses.size(); ++i)
        {
            lenPoint.first = msg->poses[i].pose.position;
            lenPoint.second = getDist(prevPoint, msg->poses[i].pose.position);
            globalPath_.push_back(lenPoint);
            pathToFollow_.push_back(lenPoint);
            prevPoint = msg->poses[i].pose.position;

            pathLenToDest_ += lenPoint.second;
        }
      //  ROS_INFO("globalPath_.x: %f ", globalPath_[0].first.x);        
    	b_gotPathMsg_ = true;
    }
    
}


void SmartVehicleSteeringClass::emBrakeCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    b_emBrake_ = msg->data;
    b_gotBrakeMsg_ = true;
}


float SmartVehicleSteeringClass::getDist(
    geometry_msgs::Point p1,
    geometry_msgs::Point p2)
{
    return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}


float SmartVehicleSteeringClass::getEnuHeading(
    geometry_msgs::Point tailPoint,
    geometry_msgs::Point headPoint)
{
    if(headPoint.y - tailPoint.y > 0)
    {
        return atan2(headPoint.y - tailPoint.y, headPoint.x - tailPoint.x);
    }
    else
    {
        return 2*M_PI + atan2(headPoint.y - tailPoint.y, headPoint.x - tailPoint.x);
    }
}


float SmartVehicleSteeringClass::getDistToLine(
    geometry_msgs::Point point, 
    geometry_msgs::Point lineP1,
    geometry_msgs::Point lineP2)
{
    float a = lineP1.y - lineP2.y;
    float b = lineP2.x - lineP1.x;
    float c = lineP1.x * lineP2.y - lineP2.x * lineP1.y;

    float d = (point.x - lineP1.x) * (lineP2.y - lineP1.y) -
        (point.y - lineP1.y) * (lineP2.x - lineP1.x);

    if(d < 0)
    {
        b_lineOnLeft_ = false;
    }
    else
    {
        b_lineOnLeft_ = true;
    }
    
    return (1-2*b_lineOnLeft_) * fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}

bool SmartVehicleSteeringClass::arePointsTheSame(
    geometry_msgs::Point p1, 
    geometry_msgs::Point p2)
{
    const float EPS = std::numeric_limits<float>::epsilon();
    if(fabs(p1.x - p2.x) < EPS && fabs(p1.y - p2.y) < EPS)
    {
        return true;
    }
    else
    {
        return false;
    }
}



void SmartVehicleSteeringClass::estimateVelocity()
{
    float dist = 0.0;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    bool foundP2 = false;
    bool foundP3 = false;

    float A = 0.0;
    float B = 0.0;
    float C = 0.0;
    float D = 0.0;

    if(localPath_.size() > 2)
    {
        p1 = localPath_[0].first;
        for(size_t i = 1; i < localPath_.size(); ++i)
        {
            dist += localPath_[i].second;
            if(dist > PATH_LEN_RADIUS && !foundP2)
            {
                p2 = localPath_[i].first;
                foundP2 = true;
            }
            if( (dist > 3.0 * PATH_LEN_RADIUS) && foundP2)
            {
                p3 = localPath_[i].first;
                foundP3 = true;
                break;
            }
        }
        if(foundP2 && foundP3 &&
            !arePointsTheSame(p1, p2) && 
            !arePointsTheSame(p2, p3) &&
            !arePointsTheSame(p1, p3))
        {
            A = p1.x*(p2.y - p3.y) - p1.y*(p2.x - p3.x) + p2.x*p3.y - p3.x*p2.y;
            B = (p1.x*p1.x + p1.y*p1.y) * (p3.y - p2.y) + (p2.x*p2.x + p2.y*p2.y) * (p1.y - p3.y) + 
                (p3.x*p3.x + p3.y*p3.y) * (p2.y - p1.y);
            C = (p1.x*p1.x + p1.y*p1.y) * (p2.x - p3.x) + (p2.x*p2.x + p2.y*p2.y) * (p3.x - p1.x) +
                (p3.x*p3.x + p3.y*p3.y) * (p1.x - p2.x);
            D = (p1.x*p1.x + p1.y*p1.y) * (p3.x*p2.y - p2.x*p3.y) + (p2.x*p2.x + p2.y*p2.y) * 
                (p1.x*p3.y - p3.x*p1.y) + (p3.x*p3.x + p3.y*p3.y) * (p2.x*p1.y - p1.x*p2.y);
            curveRadius_ = sqrt((B*B + C*C - 4.0*A*D)/(4.0*A*A));
            if(curveRadius_ > UPPER_RADIUS_VAL)
            {
                b_curveAhead = false;
            }
            b_curveAhead = true;
        }
        else
        {
            curveRadius_ = UPPER_RADIUS_VAL;
            b_curveAhead = false;
        }
    }

    if(b_curveAhead)
    {      
        velocityCmd_.data = MIN_VEL_DRIVE + AVG_VEL_DRIVE * sqrt(curveRadius_/UPPER_RADIUS_VAL);
        if(velocityCmd_.data < MIN_VEL_DRIVE)
        {
            velocityCmd_.data = MIN_VEL_DRIVE;
        }
        else if(velocityCmd_.data > MAX_VEL_DRIVE)
        {
            velocityCmd_.data = MAX_VEL_DRIVE;
        }   
    }
    else
    {
        velocityCmd_.data = MAX_VEL_DRIVE;
    }
}


void SmartVehicleSteeringClass::estimateErrors()
{
    float range = HEADING_TIME_CONS * velocityCmd_.data; //all has to be in SI!!
    float dist = 0.0;
    float diff = 0.0;
    bool b_lineOnLeft = false;
    geometry_msgs::Point headPoint;
    geometry_msgs::Point tailPoint;
    headPoint.x = globalPath_[globalPath_.size()-1].first.x;
    headPoint.y = globalPath_[globalPath_.size()-1].first.y;
    tailPoint = lastDelLocalPoint_.first;
    if(range > LOCAL_PATH_MAX_LEN)
    {
        range = LOCAL_PATH_MAX_LEN;
    }

    if(localPath_.size() > 1)
    {
        tailPoint = localPath_[0].first;
        for(size_t i = 0; i < localPath_.size(); ++i)
        {
            dist += localPath_[i].second;
            if(range <= dist || i == localPath_.size()-1)
            {
                headPoint = localPath_[i].first;
                break;
            }
        }
        dist = 0.0;
    }

    if(arePointsTheSame(tailPoint, headPoint) && globalPath_.size() > 1)
    {
        tailPoint = lastDelLocalPoint_.first;
    }

    //ROS_INFO("lastDelLocalPoint_.first: %f, %f ", lastDelLocalPoint_.first.x, lastDelLocalPoint_.first.y);
    
    
    lineHeading_ = getEnuHeading(carPos_, headPoint);
    ////????carPos_carPos_carPos_
    //ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    //ROS_INFO("tailPoint: %f, %f ", tailPoint.x, tailPoint.y);
   //ROS_INFO("headPoint: %f, %f ", headPoint.x, headPoint.y);
    
/*
    if(fmod(carHeading_ + M_PI, 2*M_PI) > lineHeading_)
    {
        b_lineOnLeft = true;
    }
    else
    {
        b_lineOnLeft = false;
    }
    if( (lineHeading_ > 3*M_PI/2.0 && lineHeading_ < 2*M_PI) && 
        carHeading_ < M_PI/2.0 )
    {
        diff = carHeading_ + 2.0*M_PI - lineHeading_;
    }
    else if( (carHeading_ > 3*M_PI/2.0 && carHeading_ < 2*M_PI) && 
        lineHeading_ < M_PI/2.0 )
    {
        diff = carHeading_ - 2.0*M_PI - lineHeading_;
    }
    else 
    {
        diff = carHeading_ - lineHeading_;   
    }
    */
    diff = carHeading_ - lineHeading_;
    headingErrorToPID_ = diff;
   // if(headingErrorToPID_ > M_PI)
   // {
        
    //}

    distErrorToPID_ = getDistToLine(carPos_, tailPoint, headPoint);
    if(distErrorToPID_ > CAR_WIDTH/2.0)
    {
        distErrorToPID_ = CAR_WIDTH/2.0;
    }
    else if(distErrorToPID_ < -CAR_WIDTH/2.0)
    {
        distErrorToPID_ = -CAR_WIDTH/2.0;
    }
    
    if(headingErrorToPID_ < -M_PI)
    {
         headingErrorToPID_ = 2*M_PI + headingErrorToPID_;
    }
    if(headingErrorToPID_ > M_PI)
    {
         headingErrorToPID_ = -2*M_PI + headingErrorToPID_;
    }

    if(arePointsTheSame(tailPoint, headPoint) )
    {
    	b_destReached_ = true;
	    headingErrorToPID_ = 0.0;
	    distErrorToPID_ = 0.0;
    }
}


void SmartVehicleSteeringClass::estimateSteeringAngle()
{
    steeringAngleCmd_.data = (-headingErrorToPID_ * KP_HEADING -
        distErrorToPID_ * M_PI/180 * KP_DIST); //* 0.001 * timeCycle_;

    if(steeringAngleCmd_.data > 8*M_PI)
    {
         steeringAngleCmd_.data = 8*M_PI;
    }
    else if(steeringAngleCmd_.data < -8*M_PI)
    {
        steeringAngleCmd_.data = -8*M_PI;
    }
}


void SmartVehicleSteeringClass::estimateThrottlePercent()
{
    int stop = 0;
    int accelerate = 1;
    int slowOnGear = 2;
    int slowOnBrake = 3;
    const float EPS = std::numeric_limits<float>::epsilon();

    rampingVelocity();



    if(rampingVelocity_ >= actualCarSpeed_)
    {
        currentThrottleState_ = accelerate;
    }
    else if(rampingVelocity_ < EPS && actualCarSpeed_ < STOP_CAR_HIST )    
    {
        currentThrottleState_ = stop;
    }
    else if(actualCarSpeed_ - rampingVelocity_ < VEL_HIST_ON_GEAR && velocityCmd_.data > EPS)
    {
        currentThrottleState_ = slowOnGear;
    }
    else if(rampingVelocity_ < actualCarSpeed_)
    {
        currentThrottleState_ = slowOnBrake;
    }
    
    ///
    if(b_destReached_)
    {
         currentThrottleState_ = stop;
       // currentThrottleState_ = slowOnBrake;
    }
    ///
    

    if(currentThrottleState_ == accelerate)
    {
        throttlePercent_ = (rampingVelocity_ - actualCarSpeed_) * KP_THROTTLE * 0.001 * timeCycle_;
        brakePercent_ = 0.001;
    }
    else if(currentThrottleState_ == slowOnBrake)
    {
        throttlePercent_ = 0.0;
        brakePercent_ = (actualCarSpeed_ - velocityCmd_.data) * KP_BRAKE * 0.001 * timeCycle_;
    }
    else if(currentThrottleState_ == stop)
    {
        throttlePercent_ = 0.0;
        brakePercent_ += MAX_BRAKE_PERCENT;//* 0.001 * timeCycle_ * 4.0;
    }
    else if(currentThrottleState_ == slowOnGear)
    {
        throttlePercent_ = 0.0;
        brakePercent_ = 0.001;
    }

    if(brakePercent_ > MAX_BRAKE_PERCENT)
    {
        brakePercent_ = MAX_BRAKE_PERCENT;
    }
    if(throttlePercent_ > MAX_THROTTLE_PERCENT)
    {
        throttlePercent_ = MAX_THROTTLE_PERCENT;
    }
    // ROS_INFO("Current throttle state: %d", currentThrottleState_);
}


void SmartVehicleSteeringClass::updateLocalPath()
{
    std::vector<std::pair <geometry_msgs::Point, float> > tempLocalPath;
    std::vector<std::pair <geometry_msgs::Point, float> > tempPathLenToDest;
     
    //ROS_INFO("localPath_.x: %f ", localPath_[0].first.x);

    for(size_t i = 0; i < localPath_.size(); ++i)
    {
        if(isBehindCar(localPath_[i].first) )
        {
            pathLenToDest_ -= pathToFollow_[i].second;
            pathToFollow_[i].second = -1.0;
        }
        else
        {
            break;
        }
    }

    for(size_t i = 0; i < pathToFollow_.size(); ++i)
    {
        if(localPath_[i].second > 0)
        {
            tempPathLenToDest.push_back(pathToFollow_[i]);
        }
    }
    pathToFollow_ = tempPathLenToDest;



    for(size_t i = 0; i < localPath_.size(); ++i)
    {
        if(isBehindCar(localPath_[i].first) )
        {
            lPathCurrentLen_ -= localPath_[i].second;
            localPath_[i].second = -1.0;
        }
        else
        {
            break;
        }
    }

    for(size_t i = 0; i < localPath_.size(); ++i)
    {
        if(localPath_[i].second > 0)
        {
            tempLocalPath.push_back(localPath_[i]);
        }
        else
        {
            ++numRemovedPt_;
            lastDelLocalPoint_ = localPath_[i];
        }
    }
    localPath_ = tempLocalPath;
    
    
    while(true)
    {
        if(lPathCurrentLen_ < LOCAL_PATH_MAX_LEN)
        {
        //    ROS_INFO("lPathCutIndex_: %d, and LOCAL_PATH_MAX_LEN: %d", lPathCutIndex_, LOCAL_PATH_MAX_LEN);
        //    ROS_INFO("lPathCurrentLen_: %d, and LOCAL_PATH_MAX_LEN: %d", lPathCurrentLen_, LOCAL_PATH_MAX_LEN);
            if(lPathCutIndex_ < globalPath_.size()-1)
            {
        //        ROS_INFO("globalPath_[lPathCutIndex_].second: %d", globalPath_[lPathCutIndex_].second);
                ++lPathCutIndex_;
                localPath_.push_back(globalPath_[lPathCutIndex_]);
                lPathCurrentLen_ += globalPath_[lPathCutIndex_].second;
            }
            else
            {
                break;
            }
        }
        else
        {
        //     ROS_INFO("lPathCurrentLen_: %d is not shorter than LOCAL_PATH_MAX_LEN: %d", lPathCutIndex_, LOCAL_PATH_MAX_LEN);        
            break;
        }   
    }
    //ROS_INFO("22localPath_.x: %f ", localPath_[0].first.x);
}

bool SmartVehicleSteeringClass::isBehindCar(
    geometry_msgs::Point point)
{
    float angleDiff = 0.0;
    angleDiff = fabs(getEnuHeading(carPos_, point) - carHeading_);
    if (angleDiff > M_PI)
    {
        angleDiff = 2*M_PI - angleDiff; 
    }
    //ROS_INFO("##################+++++++++++++++++##############################");
    //ROS_INFO("angleDiff: %f ", angleDiff * 180.0/M_PI);
   // ROS_INFO("getEnu: %f ", getEnuHeading(carPos_, point) * 180.0/M_PI);
   // ROS_INFO("carHeading_: %f ", carHeading_ * 180.0/M_PI);

    if(angleDiff > M_PI/2.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}


void SmartVehicleSteeringClass::calculateBackWheelsPositions()
{
    leftBWPos_.x = (CAR_WIDTH/2.0) * sin(carHeading_) + carPos_.x;
    leftBWPos_.y = (-CAR_WIDTH/2.0) * cos(carHeading_) + carPos_.y;
    rightBWPos_.x = (-CAR_WIDTH/2.0) * sin(carHeading_) + carPos_.x;
    rightBWPos_.y = (CAR_WIDTH/2.0) * cos(carHeading_) + carPos_.y;
}


bool SmartVehicleSteeringClass::isDestReached()
{
    float dist = getDist( carPos_, globalPath_[globalPath_.size()-1].first );
   // ROS_INFO("dist W destinationreach %f", dist);

  //  ROS_INFO("lPathCutIndex_ : %d",lPathCutIndex_);
   // ROS_INFO("globalPath_.size()-1 : %d", (globalPath_.size()-1));
    
    if ( ( lPathCutIndex_ == globalPath_.size()-1 ) && (dist < MIN_TIME_TO_DEST_POINT * actualCarSpeed_ || dist < 2.5) )
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool SmartVehicleSteeringClass::isPathCrossed()
{
    const float EPS = std::numeric_limits<float>::epsilon();

    if(localPath_.size() > 0 && numRemovedPt_ > 2 && 
        !arePointsTheSame(localPath_[0].first, lastDelLocalPoint_.first ) && actualCarSpeed_ > EPS )
    {
        float ax = localPath_[0].first.x - lastDelLocalPoint_.first.x;
        float ay = localPath_[0].first.y - lastDelLocalPoint_.first.y;

        float bx = rightBWPos_.x - leftBWPos_.x;
        float by = rightBWPos_.y - leftBWPos_.y;

        float dx = rightBWPos_.x - lastDelLocalPoint_.first.x;
        float dy = rightBWPos_.y - lastDelLocalPoint_.first.y;

        float det = ax * by - ay * bx;

        if (det == 0) 
        {
            return true;
        }

        float r = (dx * by - dy * bx) / det;
        float s = (ax * dy - ay * dx) / det;

        return (r < 0 || r > 1 || s < 0 || s > 1);
    }

    return false;
}


void SmartVehicleSteeringClass::rampingVelocity()
{
    timeCycle_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime_).count();
    if( fabs(rampingVelocity_ - velocityCmd_.data) < timeCycle_ * 0.001 * RAMP_VEL_PER_SEC )
    {
        rampingVelocity_ = velocityCmd_.data;
    }
    else
    {
        if(rampingVelocity_ < velocityCmd_.data)
        {
            rampingVelocity_ += timeCycle_ * 0.001 * RAMP_VEL_PER_SEC;
        } 
        else if(rampingVelocity_ > velocityCmd_.data)
        {
            rampingVelocity_ -= timeCycle_ * 0.001 * RAMP_VEL_PER_SEC;
        }
    }
    
    startTime_ = std::chrono::high_resolution_clock::now();
}


void SmartVehicleSteeringClass::finiteStateMachine()
{
    const float EPS = std::numeric_limits<float>::epsilon();

    int prevState = currentState_;
    path_follower_msgs::PathFollowerDebugInfo refMsg;
    if(b_emBrake_)
    {
        currentState_ = refMsg.RESET_STATE;
        b_emBrake_ = false;
    }
    
    if(currentState_ == refMsg.PREP_TO_DRIVE)
    {
        b_pathCrossed_ = false;
        velocityCmd_.data = 0.0;
        throttlePercent_ = 0.0;
        brakePercent_ = MAX_BRAKE_PERCENT;
        steeringAngleCmd_.data = 0.0;
        if(b_allowDriving_ && b_gotPathMsg_ && b_gotHeadingMsg_ )
        {
            b_newPathRequest_.data = false;
            if(globalPath_.size() > 0)
            {
                lPathCurrentLen_ = 0.0;
                for(size_t i = 0; i < globalPath_.size(); ++i)
                {
                    lPathCurrentLen_ += globalPath_[i].second;

                    if(lPathCurrentLen_ < LOCAL_PATH_MAX_LEN)
                    {
                        localPath_.push_back(globalPath_[i]);
                        lPathCutIndex_ = i;
                    }
                    else
                    {
                        break;
                    }  
                }
                if(localPath_.size() > 0)
                {
                    currentState_ = refMsg.WAIT_FOR_PATH_VAL;
                }
            }
        }
    }
    else if(currentState_ == refMsg.WAIT_FOR_PATH_VAL)
    {
        if(b_pathValid_)
        {
            currentState_ = refMsg.WAIT_FOR_GEAR;
            b_pathValid_ = false;
        }
    }
    else if(currentState_ == refMsg.WAIT_FOR_GEAR)
    {
        dbw_mkz_msgs::GearCmd gearCmd;
        gearCmd.cmd.gear = gearCmd.cmd.DRIVE;
        pubAskForGearChange.publish(gearCmd);
        if(gearState_.cmd.gear == gearState_.cmd.DRIVE)
        {
            currentState_ = refMsg.CAR_DRIVING;
        }
    }
    else if(currentState_ == refMsg.CAR_DRIVING)
    {
        if(!b_allowDriving_)
        {
            currentState_ = refMsg.PREP_TO_DRIVE;
        }
        else if(b_destReached_) 
        {
            currentState_ = refMsg.DEST_REACHED;
        }
        else
        {
            updateLocalPath();
            calculateBackWheelsPositions();
            b_pathCrossed_ = isPathCrossed();           
	        estimateVelocity();
            estimateThrottlePercent();
            estimateErrors();
            estimateSteeringAngle();
            b_destReached_ = isDestReached();
            if(b_pathCrossed_)
            {
                //currentState_ = refMsg.PATH_CROSSED;
            }
            if(b_destReached_) 
            {
                currentState_ = refMsg.DEST_REACHED;
            }
        }
    }
    else if(currentState_ == refMsg.DEST_REACHED)
    {
        velocityCmd_.data = 0.0;
        estimateThrottlePercent();
        steeringAngleCmd_.data = 0.0;
        globalPath_.clear();
        localPath_.clear();
        lPathCutIndex_ = 0;
        numRemovedPt_ = 0;
        b_gotPathMsg_ = false;
        if(actualCarSpeed_ < EPS)
        {
            currentState_ = refMsg.PREP_TO_DRIVE;
            b_destReached_ = false;
        }
    }
    else if(currentState_ == refMsg.PATH_CROSSED)
    {
        velocityCmd_.data = 0.0;
        estimateThrottlePercent();
        if(actualCarSpeed_ < EPS)
        {
            currentState_ = refMsg.PREP_TO_DRIVE;
            b_pathCrossed_ = false;
        }

        globalPath_.clear();
        localPath_.clear();
        b_gotPathMsg_ = false;
        b_newPathRequest_.data = true;
    }
    else if(currentState_ == refMsg.RESET_STATE)
    {
        velocityCmd_.data = 0.0;
        throttlePercent_ = 0.0;
        brakePercent_ = MAX_BRAKE_PERCENT;
        //estimateThrottlePercent();
        b_gotPathMsg_ = false;
        if(actualCarSpeed_ < EPS)
        {
            currentState_ = refMsg.PREP_TO_DRIVE;
        }
        globalPath_.clear();
        localPath_.clear();
        b_destReached_ = false;
    }
    if(currentState_ != prevState) 
    { 
        previousState_ = prevState;
    }
     
}

void SmartVehicleSteeringClass::publishCmds()
{
    dbw_mkz_msgs::SteeringCmd steeringMsg;
    steeringMsg.enable = true;
    steeringMsg.steering_wheel_angle_cmd = steeringAngleCmd_.data;
    pubDbwSteeringAngle.publish(steeringMsg);

    dbw_mkz_msgs::BrakeCmd brakeMsg;
    brakeMsg.enable = true;
    brakeMsg.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
    brakeMsg.pedal_cmd = brakePercent_;
    pubDbwBrake.publish(brakeMsg);

    dbw_mkz_msgs::ThrottleCmd throttleMsg;
    throttleMsg.enable = true;
    throttleMsg.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    //
    throttleMsg.pedal_cmd = throttlePercent_;
    pubDbwThrottle.publish(throttleMsg);

    pubVelocity.publish(velocityCmd_);
    pubSteeringAngle.publish(steeringAngleCmd_);

    pubNewPathRequest.publish(b_newPathRequest_);


    nav_msgs::Path pathToFollowCmd;
    geometry_msgs::PoseStamped stmpPose;
    std::pair<geometry_msgs::Point, float> lenPoint;
    for (size_t i = 0; i < pathToFollow_.size(); ++i)
    {
        stmpPose.pose.position = pathToFollow_[i].first;
        pathToFollowCmd.poses.push_back(stmpPose);
    }
    pubPathToFollow.publish(pathToFollowCmd);

    nav_msgs::Path locPath;
    geometry_msgs::PoseStamped stmpLocPose;
    locPath.header.frame_id="map";
    for (size_t i = 0; i < localPath_.size(); ++i)
    {
        stmpLocPose.pose.position = localPath_[i].first;
        locPath.poses.push_back(stmpLocPose);
    }
    pubLocalPath.publish(locPath);
}


void SmartVehicleSteeringClass::printDebLog()
{
/*
    ROS_INFO("##########################################################");
    ROS_INFO("------------flags-----------");
    ROS_INFO("b_gotPathMsg_: %d ", b_gotPathMsg_);
    ROS_INFO("b_gotHeadingMsg_: %d ", b_gotHeadingMsg_);
    ROS_INFO("b_allowDriving_: %d ", b_allowDriving_);
    ROS_INFO("b_destReached_: %d ", b_destReached_);

    ROS_INFO("b_pathCrossed_: %d ", b_pathCrossed_);
    ROS_INFO("b_lineOnLeft_: %d ", b_lineOnLeft_);
    ROS_INFO("b_newPathRequest_.data: %d ", b_newPathRequest_.data);
    ROS_INFO("------------cmds------------");
    ROS_INFO("carPos_: (%f, %f) ", carPos_.x, carPos_.y);
    ROS_INFO("left-right back wheel pos:: (%f, %f), (%f, %f)", leftBWPos_.x, leftBWPos_.y, 
        rightBWPos_.x, rightBWPos_.y);

*/
    if(carHeading_ > 0.00001 || carHeading_ < -0.00001)
    {
    ROS_INFO("##################+++++++++++++++++##############################");
    ROS_INFO("carHeading_: %f ", carHeading_ * 180.0/M_PI);
     ROS_INFO("lineHeading_: %f ", lineHeading_ * 180/M_PI);    
    ROS_INFO("steeringAngleCmd_: %f ", steeringAngleCmd_.data * 180/M_PI);
    ROS_INFO("headingErrorToPID_: %f ", headingErrorToPID_ * 180/M_PI);
    }
   // ROS_INFO("------------PID errors------------");
    
   // ROS_INFO("distErrorToPID_: %f ", distErrorToPID_);
   // ROS_INFO("------------others------------");
   // ROS_INFO("curveRadius_: %f ", curveRadius_);
    /*
    ROS_INFO("lPathCurrentLen_: %f ", lPathCurrentLen_);
    ROS_INFO("lPathCutIndex_: %d ", lPathCutIndex_);
    ROS_INFO("globalPath_.size(): %d ", int(globalPath_.size()));
    ROS_INFO("numRemovedPt_: %d ", numRemovedPt_);
    
    ROS_INFO("------------carState------------");
    ROS_INFO("currentState_: %d ", currentState_);
*/


/*
    ROS_INFO("pathLenToDest_: %f ", pathLenToDest_);
    ROS_INFO("------------velocityRegulator------------");
    ROS_INFO("currentThrottleState_: %d ", currentThrottleState_);
    ROS_INFO("throttlePercent_: %f ", throttlePercent_);
    ROS_INFO("brakePercent_: %f ", brakePercent_);
    ROS_INFO("-----------------");
    ROS_INFO("actualCarSpeed_: %f ", actualCarSpeed_);
    ROS_INFO("velocityCmd_: %f ", velocityCmd_.data);
    ROS_INFO("rampingVelocity_: %f ", rampingVelocity_);
    ROS_INFO("-----------------");
    ROS_INFO("timeCycle_: %f ", timeCycle_);
    ROS_INFO("RAMP_VEL_PER_SEC: %f ", RAMP_VEL_PER_SEC);
    ROS_INFO("CAR_WIDTH: %f ", CAR_WIDTH);
    ROS_INFO("-----------------"); 
*/
    
}

void SmartVehicleSteeringClass::publishDebugToVis()
{
    path_follower_msgs::PathFollowerDebugInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.path_received = b_gotPathMsg_;
    msg.destination_reached = b_destReached_;
    msg.path_lost = b_pathCrossed_;
    msg.brake_command = brakeCmd_.data;
    msg.current_state = currentState_;
    msg.previous_state = previousState_;
    pubDebugToRqt.publish(msg);
}
