#include "hello_gui.h"
#include "ui_hello_gui.h"
#include <QPixmap>
#include <QFile>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <iostream>
#include <QPointF>
#include <cmath>
#include <ros/console.h>
#include "logic_swift_nav/AdvertisePath.h"


HelloGui::HelloGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::HelloGui)
{
  ui->setupUi(this);
  ui->graphicsView->setMouseTracking(true);
  //ui->graphicsView->layout()->addWidget(new MyGraphicsView());

  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(on_pushClicked()));
  connect(&sc,SIGNAL(mouse_pos(QPointF)),this,SLOT(drawClickedPoint(QPointF))); // polaczenie wygnalow
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate


  // setup subscriber by according to the ~/chatter_topic param
  std::string listen_topic;
  ros::param::get("/actual_kia_position_topic", listen_topic);
  chatter_sub_ = nh_->subscribe<gps_msgs::SystemState>(listen_topic, 1, &HelloGui::xyCallback, this); // gps_msgs::SystemState azsp_msgs::Imu

  // publish a message on the channel specified by ~/hello_topic param
  std::string pub_topic;
  std::string start_topic;
  std::string enable_topic;
  ros::param::get("/start_topic", start_topic);
  start_pub_ = nh_->advertise<std_msgs::Bool>(start_topic,1);
  ros::param::get("/setpoint_topic", pub_topic);
  setpoint_pub_ = nh_->advertise<azsp_msgs::FloatPoint>(pub_topic,1);
  ros::param::get("/enable_topic", enable_topic);
  enable_pub_ = nh_->advertise<std_msgs::Bool>(enable_topic,1);
  //nh_->subscribe<dbw_mkz_msgs::onRosGearReport>("/vehicle/gear_report", 1, &HelloGui::gearCallback, this);
  enable_msg.data = false; // set as default 0 value
  clientAdvertisePath = nh_->serviceClient<logic_swift_nav::AdvertisePath>("/advertise_path"); // advertise path

  ros::param::get("/map_path", map_path);
  draw_background();
}

HelloGui::~HelloGui()
{
  delete ui;
  delete ros_timer;
}

void HelloGui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void HelloGui::xyCallback(const gps_msgs::SystemState::ConstPtr& msg){
  ui->hi_num->setValue(int(12));
  sub_x = msg->nav_sat_fix.longitude; // msg->pose.x; //
  sub_y = msg->nav_sat_fix.latitude; // msg->pose.y; //
  angle = msg->orientation_rad.heading; // msg->orientation.z*3.14/180.0; //
  sc.clear();
  drawMap();
}

// void HelloGui::onRosGearReport(dbw_mkz_msgs::GearReportConstPtr ptrMsg)
// {
//    if (ptrMsg->state.gear == 0) 
//    {
//        currentGear_ = "None";
//    } else if (ptrMsg->state.gear == 1) 
//    {
//        currentGear_ = "Park";
//    } else if (ptrMsg->state.gear == 2) 
//    {
//        currentGear_ = "Reverse";
//    } else if (ptrMsg->state.gear == 3) 
//    {
//        currentGear_ = "Neutral";
//    } else if (ptrMsg->state.gear == 4) 
//    {
//        currentGear_ = "Drive";
//    } else if (ptrMsg->state.gear == 5) 
//    {
//        currentGear_ = "Low";
//    }
//    ui_.GearLabel->setText(currentGear_);
// }

void HelloGui::on_enable_button_clicked()
{
  // std_msgs::String msg;
  // std::stringstream ss;
  // ss << "hello world " << ui->hi_num->value();
  // msg.data = ss.str();
  
  enable_msg.data = !enable_msg.data;

  enable_pub_.publish(enable_msg);
  if (enable_msg.data == 0 )
  {
    ui->enable_button->setText("Now OSCC is disabled");
  }
  else {
    ui->enable_button->setText("Now OSCC is enabled");
  }
  //ui->hi_num->setValue(ui->hi_num->value()+1);
}

void HelloGui::on_pushClicked()
{
  //advertise path
  logic_swift_nav::AdvertisePathRequest req;
  logic_swift_nav::AdvertisePathResponse res;
  req.pathFileName = "/sciezka.json";//waypointFileName.toUtf8().constData();
  if (clientAdvertisePath.call(req, res))
  {
      switch(res.status)
      {
      case logic_swift_nav::AdvertisePathResponse::FILE_OK:
          ROS_INFO("Start advertise path %s", req.pathFileName.c_str());
          break;
      case logic_swift_nav::AdvertisePathResponse::FILE_NOT_EXIST:
          ROS_ERROR("File not exist. path %s", req.pathFileName.c_str());
          break;
      case logic_swift_nav::AdvertisePathResponse::FILE_INVALID:
          ROS_ERROR("File is invalid. path %s", req.pathFileName.c_str());
          break;
      }
  }
  // start drive
  std_msgs::Bool msg;
  msg.data = true;
  start_pub_.publish(msg); 
}

void HelloGui::draw_background()
{
  QString url(QString::fromStdString(map_path)); 
  QPixmap pixmap;
  pixmap.load(url);

  ui->graphicsView->setBackgroundBrush(QBrush(pixmap));
  // set scene
  ui->graphicsView->setScene(&sc);

  //try draw something
  //sc.addRect(QRectF(0, 0, 100, 200), QPen(Qt::black), QBrush(Qt::red));
}

void HelloGui::drawClickedPoint(QPointF point)
{
  clickedPoint = point;
  int x = point.x()-10;
  int y = point.y()-10;
  const int DIAMETER = 12;
  auto circle = new QGraphicsEllipseItem(x, y, DIAMETER, DIAMETER);
  circle->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
  circle->setBrush(Qt::red);
  sc.addItem(circle);
  // set scene
  ui->graphicsView->setScene(&sc);

  // send ros message
  azsp_msgs::FloatPoint msg;
  msg.x = x;
  msg.y = y;
  setpoint_pub_.publish(msg);
}

void HelloGui::LoadCoordinateSystem()
{
  ros::param::get("/coordinate_system/xmin", xmin);
  ros::param::get("/coordinate_system/xmax", xmax);
  ros::param::get("/coordinate_system/ymin", ymin);
  ros::param::get("/coordinate_system/ymax", ymax);
  ros::param::get("/coordinate_system/image_size_x", image_size_x);
  ros::param::get("/coordinate_system/image_size_y", image_size_y);
}

void HelloGui::drawMap()
{ 
  // draw actual GPS positiion (circle)
  LoadCoordinateSystem();

  graphicsView_posiotion_x = (sub_x-xmin)/(xmax-xmin)*image_size_x;
  graphicsView_posiotion_y = image_size_y-(sub_y-ymin)/(ymax-ymin)*image_size_y;

  const int DIAMETER = 10;
  auto circle = new QGraphicsEllipseItem(graphicsView_posiotion_x-5, graphicsView_posiotion_y-5, DIAMETER, DIAMETER);
  circle->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
  circle->setBrush(Qt::green);
  sc.addItem(circle);  

  // draw arrow:
  int length_line = 14;
  int x_line = length_line * cos(angle-M_PI_2);
  int y_line = length_line * sin(angle-M_PI_2);
  itemToDraw = new QGraphicsLineItem();
  itemToDraw->setPen(QPen(Qt::red, 3, Qt::SolidLine));
  QPointF origPoint = QPointF(graphicsView_posiotion_x,graphicsView_posiotion_y);
  itemToDraw->setPos(origPoint);
  itemToDraw->setLine(0,0,x_line,y_line);
  sc.addItem(itemToDraw);

  int x_line_2 = length_line/2 * cos(angle+M_PI/3);
  int y_line_2 = length_line/2 * sin(angle+M_PI/3);
  itemToDraw = new QGraphicsLineItem();
  itemToDraw->setPen(QPen(Qt::red, 3, Qt::SolidLine));
  origPoint = QPointF(graphicsView_posiotion_x+x_line,graphicsView_posiotion_y+y_line);
  itemToDraw->setPos(origPoint);
  itemToDraw->setLine(0,0,x_line_2,y_line_2);
  sc.addItem(itemToDraw);

  x_line_2 = length_line/2 * cos(angle+2*M_PI/3);
  y_line_2 = length_line/2 * sin(angle+2*M_PI/3);
  itemToDraw = new QGraphicsLineItem();
  itemToDraw->setPen(QPen(Qt::red, 3, Qt::SolidLine));
  origPoint = QPointF(graphicsView_posiotion_x+x_line,graphicsView_posiotion_y+y_line);
  itemToDraw->setPos(origPoint);
  itemToDraw->setLine(0,0,x_line_2,y_line_2);
  sc.addItem(itemToDraw);
  
  int x = clickedPoint.x()-10;
  int y = clickedPoint.y()-10;
  circle = new QGraphicsEllipseItem(x, y, DIAMETER, DIAMETER);
  circle->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
  circle->setBrush(Qt::red);
  sc.addItem(circle);

  // set scene
  ui->graphicsView->setScene(&sc);
}

void HelloGui::draw_points()
{
  float w = 52.269056;
  float h = 21.014842;
  int x = (h-xmin)/(xmax-xmin)*image_size_x;
  int y = image_size_y-(w-ymin)/(ymax-ymin)*image_size_y;
  const int DIAMETER = 12;
  auto circle = new QGraphicsEllipseItem(x, y, DIAMETER, DIAMETER);
  circle->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
  circle->setBrush(Qt::red);
  sc.addItem(circle);


  w = 52.269036;
  h = 21.014726;
  x = (h-xmin)/(xmax-xmin)*image_size_x;
  y = image_size_y-(w-ymin)/(ymax-ymin)*image_size_y;
  circle = new QGraphicsEllipseItem(x, y, DIAMETER, DIAMETER);
  circle->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
  circle->setBrush(Qt::red);
  sc.addItem(circle);

// set scene
  ui->graphicsView->setScene(&sc);
}
