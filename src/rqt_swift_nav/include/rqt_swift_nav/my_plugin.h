#ifndef rqt_swift_nav_MY_PLUGIN_H
#define rqt_swift_nav_MY_PLUGIN_H

#include <QWidget>
#include <QJsonObject>
#include <QTimer>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_swift_nav/ui_my_plugin.h>

#include "sensor_msgs/NavSatFix.h"

#include "gps_msgs/SystemState.h"
#include "gps_msgs/UTMPosition.h"
#include "gps_msgs/SystemStatus.h"
#include "swift_nav_sbp_udp_msgs/BaseStationInfo.h"

#include "logic_swift_nav/StartPathRecord.h"
#include "logic_swift_nav/StopPathRecord.h"
#include "logic_swift_nav/AdvertisePath.h"

namespace rqt_swift_nav
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

private:
  void onRosSystemState(gps_msgs::SystemStateConstPtr ptrMsg);
  void onRosUTM(gps_msgs::UTMPositionConstPtr ptrMsg);
  void onRosNavSatFixFirst(sensor_msgs::NavSatFixConstPtr ptrMsg);
  void onRosNavSatFixSecond(sensor_msgs::NavSatFixConstPtr ptrMsg);
  void onRosBaseStationFirst(swift_nav_sbp_udp_msgs::BaseStationInfoConstPtr ptrMsg);
  void onRosBaseStationSecond(swift_nav_sbp_udp_msgs::BaseStationInfoConstPtr ptrMsg);

  void onTimerAgeOfCorrectionFirst();
  void onTimerAgeOfCorrectionSecond();
  void onRecordPathPushButtonToggle(bool state);
  void onAdvertisePathButtonToggle(bool state);

  Ui::SwiftNavWidget ui_;
  QWidget* widget_;
  QTimer timerFirst;
  QTimer timerSecond;
  QTimer timerAgeOfCorrectionFirst;
  QTimer timerAgeOfCorrectionSecond;

  QString waypointFileName = "";
  QString packagePath = "";

  double ageOfCorrectionFirst = 0.0;
  bool getMsgAgeOfCorrectionFirst = false;
  double ageOfCorrectionSecond = 0.0;
  bool getMsgAgeOfCorrectionSecond = false;

  ros::Subscriber subgpsState;
  ros::Subscriber subUtmPos;
  ros::Subscriber subNavSatFixFirst;
  ros::Subscriber subNavSatFixSecond;
  ros::Subscriber subBaseStationInfoFirst;
  ros::Subscriber subBaseStationInfoSecond;

  sensor_msgs::NavSatFix actualPositionFirst;
  sensor_msgs::NavSatFix actualPositionSecond;
  gps_msgs::UTMPosition actualPositionUtm;
  gps_msgs::SystemState actualPositionSystem;

  bool recordingStartedFlag = false;

  ros::ServiceClient clientAdvertisePath;
  ros::ServiceClient clientStartPathRecord;
  ros::ServiceClient clientStopPathRecord;


};
}  // namespace rqt_swift_nav
#endif  // rqt_swift_nav_MY_PLUGIN_H
