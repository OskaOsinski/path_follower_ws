#ifndef rqt_oscc_cmd_MY_PLUGIN_H
#define rqt_oscc_cmd_MY_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_oscc_cmd/ui_my_plugin.h>
#include <QWidget>
#include "dbw_mkz_msgs/SteeringReport.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeReport.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/ThrottleReport.h"
#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/GearReport.h"
#include <QTimer>
#include <QString>
#include <std_msgs/UInt8MultiArray.h>
#include <path_follower_msgs/PathFollowerDebugInfo.h>

namespace rqt_oscc_cmd
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  void onPlannerDebugInfo(path_follower_msgs::PathFollowerDebugInfo msg);
  QString findPlannerState(const int state, const path_follower_msgs::PathFollowerDebugInfo& msg);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
public slots:
  void onEnablePushButtonClicked();
  void onDisableButtonClicked();
  void onStartButtonToggled(bool checked);
  void onStopButtonClicked();

  void onSteeringSliderValueChanged(int value);
  void onSteeringSpinBoxValueChanged(double value);

  void onBrakeSliderValueChanged(int value);
  void onBrakeSpinBoxValueChanged(int value);

  void onThorttleSliderValueChanged(int value);
  void onThrottleSpinBoxValueChanged(int value);

  void onTimerSteering();
  void onTimerBrake();
  void onTimerThrottle();
  void onDisplayTimer();

  void onSteeringEnableCheckBoxToggled(bool checked);
  void onBrakeEnableCheckBoxToggled(bool checked);
  void onThrottleEnableCheckBoxToggled(bool checked);

  void enableSteering();
  void disableSteering();

  void enableBrake();
  void disableBrake();

  void enableThrottle();
  void disableThrottle();

private:
  void initControls();
  void onRosSteeringReport(dbw_mkz_msgs::SteeringReportConstPtr ptrMsg);
  void onRosBrakeReport(dbw_mkz_msgs::BrakeReportConstPtr ptrMsg);
  void onRosThrottleReport(dbw_mkz_msgs::ThrottleReportConstPtr ptrMsg);
  void onRosGearReport(dbw_mkz_msgs::GearReportConstPtr ptrMsg);
  void startStateCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
  void steeringCmdCallback(const dbw_mkz_msgs::SteeringCmd::ConstPtr& cmd);
  void brakeCmdCallback(const dbw_mkz_msgs::BrakeCmd::ConstPtr& cmd);
  void throttleCmdCallback(const dbw_mkz_msgs::ThrottleCmd::ConstPtr& cmd);

  void setSteeringWheelAngle(double angle);
  void setBrakePosition(double position);
  void setThrottlePosition(double position);

  Ui::OsccWidget ui_;
  QWidget* widget_;

  ros::Publisher pubEnableCmdOscc_;
  ros::Publisher pubSteeringCmdDbw_;
  ros::Publisher pubBrakeCmdDbw_;
  ros::Publisher pubThrottleCmdDbw_;
  ros::Publisher pubControllerInput_;
  ros::Publisher pubStartState_;
  ros::Publisher pubStopState_;

  ros::Subscriber subSteeringPlannerDebugInfo_;
  ros::Subscriber subSteeringReportDbw_;
  ros::Subscriber subBrakeReportDbw_;
  ros::Subscriber subThrottleReportDbw_;
  ros::Subscriber subGearReport_;
  ros::Subscriber subCanFrames_;
  ros::Subscriber subControllerOutput_;
  ros::Subscriber subStartState_;
  ros::Subscriber subSteeringCmd_;
  ros::Subscriber subBrakeCmd_;
  ros::Subscriber subThrottleCmd_;
  ros::Subscriber subControllerInitialValues_;

  dbw_mkz_msgs::SteeringCmd steeringMsg;
  dbw_mkz_msgs::BrakeCmd brakeMsg;
  dbw_mkz_msgs::ThrottleCmd throttleMsg;
  dbw_mkz_msgs::GearReport gearMsg;

  QTimer timerSteering;
  QTimer timerBrake;
  QTimer timerThrottle;
  QTimer displayTimer;

  bool watchdogSteering = false;
  bool watchdogBrake = false;
  bool watchdogThrottle = false;

  bool steeringEnabled_ = false;
  bool brakeEnabled_ = false;
  bool throttleEnabled_ = false;

  double steeringAngle_ = 0.0;
  double currentSteeringTorque_ = 0.0;
  double currentSteeringAngle_ = 0.0;
  double steeringError_;

  double brake_ = 0.0;
  int brakeIndicator_ = 0;
  double currentBrakePosition_ = 0.0;
  double currentBrakePressure_ = 0.0;
  bool areBrakeLightsActive_ = false;
  double brakeError_  = 0.0;

  double throttle_ = 0.0;
  int throttleIndicator_ = 0;
  double throttleError_ = 0.0;
  double currentThrottlePosition_ = 0.0;

  bool isAutonomousModeOn = false;

  double dbwSteeringCmd = 0.0;
  double dbwBrakeCmd = 0.0;
  double dbwThrottleCmd = 0.0;

  QString currentGear_;


};
}  // namespace rqt_oscc_cmd
#endif  // rqt_oscc_cmd_MY_PLUGIN_H
