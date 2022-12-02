#include <memory>
#include <functional>
#include "rqt_swift_nav/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <cmath>
#include <limits>
#include <QMessageBox>
#include <QApplication>
#include <ros/package.h>
#include <QDir>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonArray>
#include <QVector2D>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace rqt_swift_nav
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
{
  setObjectName("MyPlugin");

  actualPositionSecond.status.status = 127;
  actualPositionFirst.status.status = 127;
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  QStringList argv = context.argv();
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  // NOTE: Here names are absolute because rqt wrap every plugin into its own namespace
  subgpsState = getNodeHandle().subscribe("/gps_state", 1, &MyPlugin::onRosSystemState, this);
  subUtmPos = getNodeHandle().subscribe("/utm_position", 1, &MyPlugin::onRosUTM, this);
  subNavSatFixFirst = getNodeHandle().subscribe("/gps_left_position", 1, &MyPlugin::onRosNavSatFixFirst, this);
  subNavSatFixSecond = getNodeHandle().subscribe("/gps_right_position", 1, &MyPlugin::onRosNavSatFixSecond, this);
  subBaseStationInfoFirst = getNodeHandle().subscribe("/gps_base_status_left", 1, &MyPlugin::onRosBaseStationFirst, this);
  subBaseStationInfoSecond = getNodeHandle().subscribe("/gps_base_status_right", 1, &MyPlugin::onRosBaseStationSecond, this);

  connect(&timerAgeOfCorrectionFirst, &QTimer::timeout, this, &MyPlugin::onTimerAgeOfCorrectionFirst);
  connect(&timerAgeOfCorrectionSecond, &QTimer::timeout, this, &MyPlugin::onTimerAgeOfCorrectionSecond);

  timerAgeOfCorrectionFirst.setInterval(5000);
  timerAgeOfCorrectionFirst.start();
  timerAgeOfCorrectionSecond.setInterval(5000);
  timerAgeOfCorrectionSecond.start();

  connect(ui_.recordPathPushButton, &QPushButton::clicked, this, &MyPlugin::onRecordPathPushButtonToggle);
  connect(ui_.advertisePathButton, &QPushButton::clicked, this, &MyPlugin::onAdvertisePathButtonToggle);

  clientAdvertisePath = getNodeHandle().serviceClient<logic_swift_nav::AdvertisePath>("/advertise_path");
  clientStopPathRecord = getNodeHandle().serviceClient<logic_swift_nav::StopPathRecord>("/stop_path_record");
  clientStartPathRecord = getNodeHandle().serviceClient<logic_swift_nav::StartPathRecord>("/start_path_record");

  context.addWidget(widget_);
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
    subgpsState.shutdown();
    subNavSatFixFirst.shutdown();
    subNavSatFixSecond.shutdown();
    subBaseStationInfoFirst.shutdown();
    subBaseStationInfoSecond.shutdown();

    clientStopPathRecord.shutdown();
    clientStartPathRecord.shutdown();
    clientAdvertisePath.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
    // v = instance_settings.value(k)
}

void MyPlugin::onRosSystemState(gps_msgs::SystemStateConstPtr ptrMsg)
{
    actualPositionSystem = *ptrMsg;
    QTimer* timer = new QTimer();
    timer->moveToThread(qApp->thread());
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]() {
        ui_.roverHeadingvalueLabel->setText(
                    QString::number(ptrMsg->orientation_rad.heading * 180 / M_PI)  );
    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::onRosUTM(gps_msgs::UTMPositionConstPtr ptrMsg)
{
    actualPositionUtm = *ptrMsg;
}

void MyPlugin::onRosNavSatFixFirst(sensor_msgs::NavSatFixConstPtr ptrMsg)
{
    QTimer* timer = new QTimer();
    timer->moveToThread(qApp->thread());
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]() {
        ui_.roverLongValueLabel->setText( QString::number(ptrMsg->longitude, 'g', 8) );
        ui_.roverLatValueLabel->setText( QString::number(ptrMsg->latitude, 'g', 8) );
        ui_.roverAltValueLabel->setText( QString::number(ptrMsg->altitude, 'g', 8) );
        ui_.roverHorAccValueLabel->setText( QString::number(ptrMsg->position_covariance[0]/ 10.0) );

    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
    if (actualPositionFirst.status.status != ptrMsg->status.status)
    {
        QString label;
        QString style;
        switch (ptrMsg->status.status)
        {
        case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
        {
            label = "NO FIX";
            style = "QLabel { background-color : red; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_FIX:
        {
            label = "2D/SPP/Float RTK fix";
            style = "QLabel { background-color : yellow; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
        {
            label = "Fixed RTK";
            style = "QLabel { background-color : green; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
        {
            label = "SBAS fix";
            style = "QLabel { background-color : yellow; color : black; }";
        }break;
            default : ;
        };

        if (label != ui_.roverFixValueLabel->text())
        {
            ui_.roverFixValueLabel->setText(label);
            ui_.roverFixValueLabel->setStyleSheet(style);
        }
    }
    actualPositionFirst = *ptrMsg;
}

void MyPlugin::onRosNavSatFixSecond(sensor_msgs::NavSatFixConstPtr ptrMsg)
{
    QTimer* timer = new QTimer();
    timer->moveToThread(qApp->thread());
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]() {
        ui_.roverLongBackValueLabel->setText( QString::number(ptrMsg->longitude, 'g', 8) );
        ui_.roverLatBackValueLabel->setText( QString::number(ptrMsg->latitude, 'g', 8) );
        ui_.roverAltBackValueLabel->setText( QString::number(ptrMsg->altitude, 'g', 8) );
        ui_.roverHorAccBackValueLabel->setText( QString::number(ptrMsg->position_covariance[0] / 10.0) );
    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

    if (actualPositionSecond.status.status != ptrMsg->status.status)
    {
        QString label;
        QString style;
        switch (ptrMsg->status.status)
        {
        case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
        {
            label = "NO FIX";
            style = "QLabel { background-color : red; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_FIX:
        {
            label = "2D/SPP/Float RTK fix";
            style = "QLabel { background-color : yellow; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
        {
            label = "Fixed RTK";
            style = "QLabel { background-color : green; color : black; }";
        }break;
        case  sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
        {
            label = "SBAS fix";
            style = "QLabel { background-color : yellow; color : black; }";
        }break;
            default : ;
        };
        if (label != ui_.roverFixBackValueLabel->text())
        {
            ui_.roverFixBackValueLabel->setText(label);
            ui_.roverFixBackValueLabel->setStyleSheet(style);
        }
    }
    actualPositionFirst = *ptrMsg;
    actualPositionSecond = *ptrMsg;
}

void MyPlugin::onRosBaseStationFirst(swift_nav_sbp_udp_msgs::BaseStationInfoConstPtr ptrMsg)
{
    ageOfCorrectionFirst = ptrMsg->age_of_correction;
    getMsgAgeOfCorrectionFirst = true;
    QTimer* timer = new QTimer();
    timer->moveToThread(qApp->thread());
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]() {
        ui_.baseAgeOfCorrectionValueLabel->setText( QString::number(ageOfCorrectionFirst) );
    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::onRosBaseStationSecond(swift_nav_sbp_udp_msgs::BaseStationInfoConstPtr ptrMsg)
{
    ageOfCorrectionSecond = ptrMsg->age_of_correction;
    getMsgAgeOfCorrectionSecond = true;
    QTimer* timer = new QTimer();
    timer->moveToThread(qApp->thread());
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]() {
        ui_.baseAgeOfCorrectionBackValueLabel->setText( QString::number(ageOfCorrectionSecond) );
    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::onTimerAgeOfCorrectionFirst()
{
    if (!getMsgAgeOfCorrectionFirst)
    {
        ui_.baseAgeOfCorrectionValueLabel->setText( "Timeout" );
    }
    getMsgAgeOfCorrectionFirst = false;

}

void MyPlugin::onTimerAgeOfCorrectionSecond()
{
    if (!getMsgAgeOfCorrectionSecond)
    {
        ui_.baseAgeOfCorrectionBackValueLabel->setText( "Timeout" );
    }
    getMsgAgeOfCorrectionSecond = false;
}

void MyPlugin::onRecordPathPushButtonToggle(bool state)
{
    if (state)
    {
        QFileDialog dialog;
        dialog.setFileMode(QFileDialog::AnyFile);
        dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
        packagePath = QString::fromStdString(ros::package::getPath("rqt_swift_nav"));
        dialog.setDirectory(packagePath);

        if (dialog.exec()) {
            waypointFileName = dialog.selectedFiles().first();
            waypointFileName.replace(packagePath, "");
        } else {
            ROS_WARN("You must choose file to save result.");
            return;
        }

        logic_swift_nav::StartPathRecordRequest req;
        logic_swift_nav::StartPathRecordResponse res;
        req.pathFileName = waypointFileName.toStdString();
        if (clientStartPathRecord.call(req, res)) {
            if (res.ifStarted) {
                recordingStartedFlag = true;
                QTimer* timer = new QTimer();
                timer->moveToThread(qApp->thread());
                timer->setSingleShot(true);
                QObject::connect(timer, &QTimer::timeout, [=]() {
                    QString style =  "QLabel { background-color : yellow; color : black; }";
                    ui_.recordPathPushButton->setStyleSheet(style);
                    ui_.recordPathPushButton->setText("Recording");
                });
                QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
            }
        } else {
            ROS_ERROR("Failed to call service");
        }

    }
    else
    {
        logic_swift_nav::StopPathRecordRequest req;
        logic_swift_nav::StopPathRecordResponse res;
        if (clientStopPathRecord.call(req, res))
        {
            QFile waypointFile(packagePath+waypointFileName);
            switch(res.status)
            {
                case logic_swift_nav::StopPathRecordResponse::STATUS_OK:
                    if (!waypointFile.open(QIODevice::WriteOnly)) {
                        qWarning("Couldn't open save file on visualization device.");
                        break;
                    }
                    waypointFile.write(res.waypoints_data.c_str());
                    res.waypoints_data;
                    break;
                case logic_swift_nav::StopPathRecordResponse::STATUS_LOST_FIX:
                    ROS_ERROR("Fix was lost during recording");
                    break;
                case logic_swift_nav::StopPathRecordResponse::STATUS_LOST_UTM_ZONE_BORDER:
                    ROS_ERROR("UTM zone border detected");
                    break;
                case logic_swift_nav::StopPathRecordResponse::STATUS_INVALID_FILE:
                    ROS_ERROR("File was invalid");
                    break;
            }
            recordingStartedFlag = false;
                QString style =  "QLabel { background-color : yellow; color : black; }";
                ui_.recordPathPushButton->setStyleSheet(style);
                ui_.recordPathPushButton->setText("Stop");
        } else {
            ROS_ERROR("Failed to call service");
        }
    }
}

void MyPlugin::onAdvertisePathButtonToggle(bool state)
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptOpen);
    QString path = QString::fromStdString(ros::package::getPath("rqt_swift_nav"));
    dialog.setDirectory(path);

    QString waypointFileName;
    if (dialog.exec())
    {
        waypointFileName = dialog.selectedFiles().first();
        waypointFileName.replace(path, "");
    } else {
        ROS_WARN("No file selected. Path will be not advertise!");
        return;
    }

    logic_swift_nav::AdvertisePathRequest req;
    logic_swift_nav::AdvertisePathResponse res;
    ROS_INFO("req: %s",req);
    ROS_INFO("res: %s",res);
    req.pathFileName = waypointFileName.toUtf8().constData();
    ROS_INFO("res: %s",req.pathFileName.c_str());
    if (clientAdvertisePath.call(req, res))
    {
        ROS_INFO("res: %s",res);
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
    else
    {
        ROS_ERROR("Failed to call service");
    }
}

}  // namespace rqt_swift_nav
PLUGINLIB_EXPORT_CLASS(rqt_swift_nav::MyPlugin, rqt_gui_cpp::Plugin)
