#ifndef HELLO_GUI_H
#define HELLO_GUI_H

#include <QWidget>
#include <QGraphicsScene>
#include <ros/ros.h>
#include <qtimer.h>
#include <string>
#include <std_msgs/String.h>
#include <azsp_msgs/FloatPoint.h>
#include <std_msgs/Bool.h>
#include <gps_msgs/SystemState.h>
#include <QGraphicsLineItem>
#include "graphicsscene.h"
#include "dbw_mkz_msgs/GearReport.h"
#include "azsp_msgs/Imu.h"

namespace Ui {
class HelloGui;
}

class HelloGui : public QWidget
{
  Q_OBJECT

public:
  explicit HelloGui(QWidget *parent = nullptr);
  ~HelloGui();
  void xyCallback(const gps_msgs::SystemState::ConstPtr& msg);
  void drawMap();
  void draw_points();
  void draw_background();
  void LoadCoordinateSystem();

public slots:
  void spinOnce();
  void drawClickedPoint(QPointF point);

private slots:
  void on_enable_button_clicked();
  void on_pushClicked();

private:
  Ui::HelloGui *ui;
  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;
  ros::Subscriber chatter_sub_;
  ros::Publisher  start_pub_;
  ros::Publisher  setpoint_pub_;
  ros::Publisher  enable_pub_;

  GraphicsScene sc;
  QGraphicsLineItem* itemToDraw;
  float sub_x;
  float sub_y;
  float angle;
  int graphicsView_posiotion_x;
  int graphicsView_posiotion_y;
  QPointF clickedPoint;
  std::string map_path;
  std::string backmap_path;
  float xmin;
  float xmax;
  float ymin;
  float ymax;
  float image_size_x;
  float image_size_y;
  std_msgs::Bool enable_msg;
  ros::ServiceClient clientAdvertisePath;
};

#endif // HELLO_GUI_H
