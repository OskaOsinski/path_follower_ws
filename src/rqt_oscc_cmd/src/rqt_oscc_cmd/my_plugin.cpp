#include "rqt_oscc_cmd/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <cmath>
#include <limits>
#include <QElapsedTimer>
#include <QDockWidget>
#include <QDebug>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>

bool AreSame(double a, double b) {
    return std::fabs(a - b) < std::numeric_limits<double>::epsilon() * 10.0;
}

double map(double val, double min1, double max1, double min2, double max2)
{
    return (val-min1)*(max2-min2)/(max1-min1) + min2;
}

namespace rqt_oscc_cmd
{

MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(nullptr)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);

    pubEnableCmdOscc_ = getNodeHandle().advertise<std_msgs::Bool>("/vehicle/oscc_enable_cmd", 1);
    pubSteeringCmdDbw_ = getNodeHandle().advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
    pubBrakeCmdDbw_ = getNodeHandle().advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);
    pubThrottleCmdDbw_ = getNodeHandle().advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);
    pubStartState_ = getNodeHandle().advertise<std_msgs::Bool>("/valid_path", 1);
    pubStopState_ = getNodeHandle().advertise<std_msgs::Bool>("/stop_button_from_rqt", 1);

    subSteeringReportDbw_ = getNodeHandle().subscribe("/vehicle/steering_report", 1, &MyPlugin::onRosSteeringReport, this);
    subBrakeReportDbw_ = getNodeHandle().subscribe("/vehicle/brake_report", 1, &MyPlugin::onRosBrakeReport, this);
    subThrottleReportDbw_ = getNodeHandle().subscribe("/vehicle/throttle_report", 1, &MyPlugin::onRosThrottleReport, this);
    subGearReport_ = getNodeHandle().subscribe("/vehicle/gear_report", 1, &MyPlugin::onRosGearReport, this);

    subSteeringPlannerDebugInfo_ = getNodeHandle().subscribe<path_follower_msgs::PathFollowerDebugInfo>
            ("/planner_debug_info", 1, &MyPlugin::onPlannerDebugInfo, this);

    subStartState_ = getNodeHandle().subscribe<std_msgs::UInt8MultiArray>("/start_state_topic", 1, &MyPlugin::startStateCallback, this);
    subSteeringCmd_ = getNodeHandle().subscribe<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1, &MyPlugin::steeringCmdCallback, this);
    subBrakeCmd_ = getNodeHandle().subscribe<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1, &MyPlugin::brakeCmdCallback, this);
    subThrottleCmd_ = getNodeHandle().subscribe<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1, &MyPlugin::throttleCmdCallback, this);

    initControls();

    connect(&timerSteering, &QTimer::timeout, this, &MyPlugin::onTimerSteering);
    connect(&timerBrake, &QTimer::timeout, this, &MyPlugin::onTimerBrake);
    connect(&timerThrottle, &QTimer::timeout, this, &MyPlugin::onTimerThrottle);
    connect(&displayTimer, &QTimer::timeout, this, &MyPlugin::onDisplayTimer);

    timerSteering.setInterval(500);
    timerSteering.start();
    timerBrake.setInterval(500);
    timerBrake.start();
    timerThrottle.setInterval(500);
    timerThrottle.start();
    displayTimer.setInterval(500);
    displayTimer.start();

    // add widget to the user interface
    context.addWidget(widget_);
}

void MyPlugin::shutdownPlugin()
{
    pubEnableCmdOscc_.shutdown();
    pubSteeringCmdDbw_.shutdown();
    pubBrakeCmdDbw_.shutdown();
    pubThrottleCmdDbw_.shutdown();
    pubControllerInput_.shutdown();
    pubStartState_.shutdown();
    pubStopState_.shutdown();

    subSteeringReportDbw_.shutdown();
    subBrakeReportDbw_.shutdown();
    subThrottleReportDbw_.shutdown();
    subCanFrames_.shutdown();
    subControllerOutput_.shutdown();
    subStartState_.shutdown();
    subSteeringPlannerDebugInfo_.shutdown();
    subSteeringCmd_.shutdown();
    subBrakeCmd_.shutdown();
    subThrottleCmd_.shutdown();
    subControllerInitialValues_.shutdown();

    timerSteering.stop();
    timerBrake.stop();
    timerThrottle.stop();
    displayTimer.stop();
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

void MyPlugin::onPlannerDebugInfo(path_follower_msgs::PathFollowerDebugInfo passedMsg)
{
    static QVector<QString> currState = {"Prepare to drive",
                                         "Car is driving",
                                         "Destination reached",
                                         "Path got lost",
                                         "Undefined error"
    };


    QString currentState  = findPlannerState(passedMsg.current_state, passedMsg);
    QString previousState = findPlannerState(passedMsg.previous_state, passedMsg);

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.labelPathPlannerState->setText(currentState);
        ui_.labelPreviousState->setText(previousState);
        ui_.labelPathReceived->setText(passedMsg.path_received ? "Yes" : "No");
        ui_.labelPathLost->setText(passedMsg.path_lost ? "Yes" : "No");
        ui_.labelBrakeCommand->setText(passedMsg.brake_command ? "Yes" : "No");
        ui_.labelDestReached->setText(passedMsg.destination_reached ? "Yes" : "No");
        ui_.labelDestReached->setText(passedMsg.destination_reached ? "Yes" : "No");
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

QString MyPlugin::findPlannerState(const int state, const path_follower_msgs::PathFollowerDebugInfo& msg)
{
    QString stateText("Undefined state");
    switch(state)
    {
        case msg.PREP_TO_DRIVE:
            stateText = "Prepare to drive";
            break;
        case msg.CAR_DRIVING:
            stateText = "Car is driving";
            break;
        case msg.DEST_REACHED:
            stateText = "Destination point reached";
            break;
        case msg.PATH_CROSSED:
            stateText = "Path got crossed error";
            break;
        case msg.WAIT_FOR_PATH_VAL:
            stateText = "Wait for path validation";
            break;
        case msg.RESET_STATE:
            stateText = "Reset state machine";
            break;
        case msg.WAIT_FOR_GEAR:
            stateText = "Wait for gear";
            break;
    }
    return stateText;
}

void MyPlugin::onEnablePushButtonClicked()
{

    std_msgs::Bool enableMsg;
    enableMsg.data = true;
    pubEnableCmdOscc_.publish(enableMsg);

    enableSteering();
    enableBrake();
    enableThrottle();

    steeringAngle_ = currentSteeringAngle_;
    steeringMsg.steering_wheel_angle_cmd = steeringAngle_;
    pubSteeringCmdDbw_.publish(steeringMsg);

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.SteeringSpinBox->setValue(steeringAngle_);
        ui_.SteeringSlider->setValue((steeringAngle_ * (double(ui_.SteeringSlider->maximum())/2.0)/ui_.SteeringSpinBox->maximum()) + (double(ui_.SteeringSlider->maximum()))/2.0);
        ui_.EnableButton->setEnabled(false);
        ui_.EnableButton->setText("OSCC enabled");
        ui_.DisableButton->setEnabled(true);
        ui_.DisableButton->setText("Disable OSCC");
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::onDisableButtonClicked()
{
    std_msgs::Bool enableMsg;
    enableMsg.data = false;
    pubEnableCmdOscc_.publish(enableMsg);

    setBrakePosition(0);
    setThrottlePosition(0);

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.EnableButton->setEnabled(true);
        ui_.EnableButton->setText("Enable OSCC");
        ui_.DisableButton->setEnabled(false);
        ui_.DisableButton->setText("OSCC disabled");
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::onStartButtonToggled(bool checked)
{
    onEnablePushButtonClicked();

    std_msgs::Bool msg;
    msg.data = true;
    pubStartState_.publish(msg);

}

void MyPlugin::onStopButtonClicked()
{
    std_msgs::Bool msg;
    msg.data = true;
    pubStopState_.publish(msg);

}

void MyPlugin::onSteeringSliderValueChanged(int value)
{
    double newAngle = (value - (ui_.SteeringSlider->maximum())/2.0) * 0.02;
    if (! AreSame(steeringAngle_, newAngle))
    {
        steeringAngle_ = newAngle;
        steeringMsg.steering_wheel_angle_cmd = steeringAngle_;
        pubSteeringCmdDbw_.publish(steeringMsg);
        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.SteeringSpinBox->setValue(steeringAngle_);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
    }
}

void MyPlugin::onSteeringSpinBoxValueChanged(double value)
{
    if (! AreSame(steeringAngle_, value))
    {
        steeringAngle_ = value;
        steeringMsg.steering_wheel_angle_cmd = steeringAngle_;
        pubSteeringCmdDbw_.publish(steeringMsg);
        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.SteeringSlider->setValue((steeringAngle_ * (double(ui_.SteeringSlider->maximum())/2.0)/ui_.SteeringSpinBox->maximum()) + (double(ui_.SteeringSlider->maximum()))/2.0);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
    }

}


void MyPlugin::onBrakeSliderValueChanged(int value)
{
    if (brakeIndicator_ != value)
    {
        brakeIndicator_ = value;
        brake_ = brakeIndicator_ / 100.0;

        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.BrakeSpinBox->setValue(brakeIndicator_);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

        brakeMsg.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
        brakeMsg.pedal_cmd = brake_;
        pubBrakeCmdDbw_.publish(brakeMsg);

    }
}

void MyPlugin::onBrakeSpinBoxValueChanged(int value)
{
    if (brakeIndicator_ != value)
    {
        brakeIndicator_ = value;
        brake_ = brakeIndicator_ / 100.0;

        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.BrakeSpinBox->setValue(brakeIndicator_);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

        brakeMsg.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
        brakeMsg.pedal_cmd = brake_;
        pubBrakeCmdDbw_.publish(brakeMsg);
    }
}


void MyPlugin::onThorttleSliderValueChanged(int value)
{
    if (throttleIndicator_ != value)
    {
        throttleIndicator_ = value;double
        throttle_ = throttleIndicator_ / 100.0;

        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.ThrottleSpinBox->setValue(throttleIndicator_);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

        throttleMsg.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
        throttleMsg.pedal_cmd = throttle_;
        pubThrottleCmdDbw_.publish(throttleMsg);
    }
}

void MyPlugin::onThrottleSpinBoxValueChanged(int value)
{
    if (throttleIndicator_ != value)
    {
        throttleIndicator_ = value;
        throttle_ = throttleIndicator_ / 100.0;

        QTimer* rqtOsccTimer = new QTimer();
        rqtOsccTimer->moveToThread(qApp->thread());
        rqtOsccTimer->setSingleShot(true);
        QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
            ui_.ThrottleSlider->setValue(throttleIndicator_);
        });
        QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

        throttleMsg.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
        throttleMsg.pedal_cmd = throttle_;
        pubThrottleCmdDbw_.publish(throttleMsg);
    }
}


void MyPlugin::onTimerSteering()
{
    QString label = "Steering: report timeout.";
    if (!watchdogSteering && ui_.steeringStateLabel->text() != label) {
        ui_.steeringStateLabel->setText(label);
        ui_.steeringStateLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    watchdogSteering = false;
}

void MyPlugin::onTimerBrake()
{
    QString label = "Brake: report timeout.";
    if (!watchdogBrake && ui_.BrakeStateLabel->text() != label) {
        ui_.BrakeStateLabel->setText(label);
        ui_.BrakeStateLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    watchdogBrake = false;
}

void MyPlugin::onTimerThrottle()
{
    QString label = "Throttle: report timeout.";
    if (!watchdogThrottle && ui_.ThrottleLabel->text() != label) {
        ui_.ThrottleLabel->setText(label);
        ui_.ThrottleLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    watchdogThrottle = false;
}

void MyPlugin::onDisplayTimer()
{
    steeringError_ = dbwSteeringCmd - currentSteeringAngle_;
    ui_.SteeringErrorLabel->setText(QString::number(steeringError_, 'f', 2) + " rad");
    ui_.SteeringActualPositionLabel->setText(QString::number(currentSteeringAngle_, 'f', 2) + " rad");
    ui_.SteeringTorqueLabel->setText(QString::number(currentSteeringTorque_, 'f', 2) + " Nm");
    brakeError_ = (dbwBrakeCmd * 100) - currentBrakePosition_;
    ui_.BrakeErrorLabel->setText(QString::number(brakeError_, 'f', 2));
    ui_.BrakeOutputPositionLabel->setText(QString::number(currentBrakePosition_) + " %");
    ui_.BrakePressureLabel->setText(QString::number(currentBrakePressure_) + " bar");
    throttleError_ = (dbwThrottleCmd * 100) - currentThrottlePosition_;
    ui_.ThrottleErrorLabel->setText(QString::number(throttleError_, 'f', 2));
    ui_.ThrottleOutputPositionLabel->setText(QString::number(currentThrottlePosition_, 'f', 2) + " %");
    ui_.GearLabel->setText(currentGear_);
    ui_.SteeringCmdLabel->setText(QString::number(dbwSteeringCmd) + " rad");
    ui_.BrakeCmdLabel->setText (QString::number(dbwBrakeCmd * 100) + " %");
    ui_.ThrottleCmdLabel->setText(QString::number(dbwThrottleCmd * 100) + " %");
}

void MyPlugin::onSteeringEnableCheckBoxToggled(bool checked)
{
    enableSteering();
    ui_.SteeringSlider->setEnabled(checked);
    ui_.SteeringSpinBox->setEnabled(checked);
}

void MyPlugin::onBrakeEnableCheckBoxToggled(bool checked)
{
    enableBrake();
    ui_.BrakeSlider->setEnabled(checked);
    ui_.BrakeSpinBox->setEnabled(checked);
}

void MyPlugin::onThrottleEnableCheckBoxToggled(bool checked)
{
    enableThrottle();
    ui_.ThrottleSlider->setEnabled(checked);
    ui_.ThrottleSpinBox->setEnabled(checked);
}

void MyPlugin::initControls()
{
    connect(ui_.EnableButton,    &QPushButton::clicked, this, &MyPlugin::onEnablePushButtonClicked);
    connect(ui_.DisableButton,   &QPushButton::clicked, this, &MyPlugin::onDisableButtonClicked);
    connect(ui_.StartPushButton, &QPushButton::toggled, this, &MyPlugin::onStartButtonToggled);
    connect(ui_.StopPushButton,  &QPushButton::clicked, this, &MyPlugin::onStopButtonClicked);

    connect(ui_.SteeringSlider, &QSlider::valueChanged, this, &MyPlugin::onSteeringSliderValueChanged);
    connect(ui_.SteeringSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &MyPlugin::onSteeringSpinBoxValueChanged);

    connect(ui_.BrakeSlider, &QSlider::valueChanged, this, &MyPlugin::onBrakeSliderValueChanged);
    connect(ui_.BrakeSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MyPlugin::onBrakeSpinBoxValueChanged);

    connect(ui_.ThrottleSlider, &QSlider::valueChanged, this, &MyPlugin::onThorttleSliderValueChanged);
    connect(ui_.ThrottleSpinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MyPlugin::onThrottleSpinBoxValueChanged);

    connect(ui_.SteeringEnableCheckBox, &QCheckBox::toggled, this, &MyPlugin::onSteeringEnableCheckBoxToggled);
    connect(ui_.BrakeEnableCheckBox, &QCheckBox::toggled, this, &MyPlugin::onBrakeEnableCheckBoxToggled);
    connect(ui_.ThrottleEnableCheckBox, &QCheckBox::toggled, this, &MyPlugin::onThrottleEnableCheckBoxToggled);

    ui_.SteeringActualPositionLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.SteeringErrorLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.SteeringTorqueLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.BrakeOutputPositionLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.BrakePressureLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.BrakeErrorLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.ThrottleOutputPositionLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.ThrottleErrorLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.GearLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.SteeringCmdLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.BrakeCmdLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");
    ui_.ThrottleCmdLabel->setStyleSheet("QLabel {background-color : white; color : dark gray; }");

    ui_.BrakeSpinBox->setSuffix(" %");
    ui_.ThrottleSpinBox->setSuffix(" %");

    ui_.tabWidget2->setCurrentIndex(0);

    ui_.EnableButton->setEnabled(true);

    ui_.SteeringCmdTextLabel->setVisible(false);
    ui_.SteeringCmdLabel->setVisible(false);
    ui_.BrakeCmdTextLabel->setVisible(false);
    ui_.BrakeCmdLabel->setVisible(false);
    ui_.ThrottleCmdTextLabel->setVisible(false);
    ui_.ThrottleCmdLabel->setVisible(false);
}
void MyPlugin::onRosSteeringReport(dbw_mkz_msgs::SteeringReportConstPtr ptrMsg)
{
    QString label = "Steering: ";
    QString style;

    if (ptrMsg->enabled)
    {
        label += " enabled";
        style = "QLabel { background-color : yellow; color : black; }";
    }
    else
    {
        label += " disabled";
        style = "QLabel { background-color : green; color : black; }";
    }
    if (ptrMsg->override)
    {
        label += " OVERIDE";
        style = "QLabel { background-color : red; color : white; }";
    }

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        if (ui_.steeringStateLabel->text() != label)
        {
            ui_.steeringStateLabel->setText(label);
            ui_.steeringStateLabel->setStyleSheet(style);
        }
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
    watchdogSteering = true;
    steeringEnabled_ = ptrMsg->enabled;
    currentSteeringAngle_ = ptrMsg->steering_wheel_angle;
}

void MyPlugin::onRosBrakeReport(dbw_mkz_msgs::BrakeReportConstPtr ptrMsg)
{
    QString label = "Brake: ";
    QString style;

    if (ptrMsg->enabled)
    {
        label += " enabled";
        style = "QLabel { background-color : yellow; color : black; }";
    }
    else
    {
        label += " disabled";
        style = "QLabel { background-color : green; color : black; }";
    }
    if (ptrMsg->override)
    {
        label += " OVERIDE";
        style = "QLabel { background-color : red; color : white; }";
    }

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        if (ui_.BrakeStateLabel->text() != label)
        {
            ui_.BrakeStateLabel->setText(label);
            ui_.BrakeStateLabel->setStyleSheet(style);
        }
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
    watchdogBrake = true;

    brakeEnabled_ = ptrMsg->enabled;
    currentBrakePosition_ = map(ptrMsg->pedal_output, 0.15, 0.50, 0.0, 100.0);
}

void MyPlugin::onRosThrottleReport(dbw_mkz_msgs::ThrottleReportConstPtr ptrMsg)
{
    QString label = "Throttle: ";
    QString style;

    if (ptrMsg->enabled)
    {
        label += " enabled";
        style = "QLabel { background-color : yellow; color : black; }";
    }
    else
    {
        label += " disabled";
        style = "QLabel { background-color : green; color : black; }";
    }
    if (ptrMsg->override)
    {
        label += " OVERIDE";
        style = "QLabel { background-color : red; color : white; }";
    }

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        if (ui_.ThrottleLabel->text() != label)
        {
            ui_.ThrottleLabel->setText(label);
            ui_.ThrottleLabel->setStyleSheet(style);
        }
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

    watchdogThrottle = true;

    throttleEnabled_ = ptrMsg->enabled;
    currentThrottlePosition_ = map(ptrMsg->pedal_output, 0.15, 0.80, 0.0, 100.0);
}


void MyPlugin::onRosGearReport(dbw_mkz_msgs::GearReportConstPtr ptrMsg)
{
   if (ptrMsg->state.gear == 0) 
   {
       currentGear_ = "None";
   } else if (ptrMsg->state.gear == 1) 
   {
       currentGear_ = "Park";
   } else if (ptrMsg->state.gear == 2) 
   {
       currentGear_ = "Reverse";
   } else if (ptrMsg->state.gear == 3) 
   {
       currentGear_ = "Neutral";
   } else if (ptrMsg->state.gear == 4) 
   {
       currentGear_ = "Drive";
   } else if (ptrMsg->state.gear == 5) 
   {
       currentGear_ = "Low";
   }
}

void MyPlugin::startStateCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    isAutonomousModeOn = msg->data[1];
}

void MyPlugin::steeringCmdCallback(const dbw_mkz_msgs::SteeringCmd::ConstPtr& cmd)
{
    dbwSteeringCmd = cmd->steering_wheel_angle_cmd;
}

void MyPlugin::brakeCmdCallback(const dbw_mkz_msgs::BrakeCmd::ConstPtr& cmd)
{
    dbwBrakeCmd = cmd->pedal_cmd;
}

void MyPlugin::throttleCmdCallback(const dbw_mkz_msgs::ThrottleCmd::ConstPtr &cmd)
{
    dbwThrottleCmd = cmd->pedal_cmd;
}


void MyPlugin::enableSteering()
{
    steeringMsg.enable=true;

}

void MyPlugin::setSteeringWheelAngle(double angle)
{
    steeringAngle_ = angle;
    steeringMsg.steering_wheel_angle_cmd=steeringAngle_;
    pubSteeringCmdDbw_.publish(steeringMsg);

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.SteeringSlider->setValue((steeringAngle_ * (double(ui_.SteeringSlider->maximum())/2.0)/ui_.SteeringSpinBox->maximum()) + (double(ui_.SteeringSlider->maximum()))/2.0);
        ui_.SteeringSpinBox->setValue(steeringAngle_);
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::disableSteering()
{
    steeringMsg.enable = false;
    pubSteeringCmdDbw_.publish(steeringMsg);
}

void MyPlugin::enableBrake()
{
    brakeMsg.enable = true;
}

void MyPlugin::setBrakePosition(double position)
{
    brake_ = position/100;
    brakeMsg.pedal_cmd = brake_;
    brakeMsg.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PERCENT;
    pubBrakeCmdDbw_.publish(brakeMsg);

    brakeIndicator_ = brake_ * 100;

    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.BrakeSpinBox->setValue(brakeIndicator_);
        ui_.BrakeSlider->setValue(brakeIndicator_);
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

void MyPlugin::disableBrake()
{
    brakeMsg.enable = false;
    pubBrakeCmdDbw_.publish(brakeMsg);
}

/////////////////////////////////////////////////////////////////// THROTTLE TEST FUNCTIONS /////////////////////////////////////////////////////////////////////

void MyPlugin::enableThrottle()
{
    throttleMsg.enable = true;
}

void MyPlugin::setThrottlePosition(double position)
{
    throttle_ = position/100;
    throttleMsg.pedal_cmd = throttle_;
    throttleMsg.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    pubThrottleCmdDbw_.publish(throttleMsg);

    throttleIndicator_ = position;
    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.ThrottleSlider->setValue(throttleIndicator_);
        ui_.ThrottleSpinBox->setValue(throttleIndicator_);
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}


void MyPlugin::disableThrottle()
{
    throttleMsg.enable = false;
    pubThrottleCmdDbw_.publish(throttleMsg);
    QTimer* rqtOsccTimer = new QTimer();
    rqtOsccTimer->moveToThread(qApp->thread());
    rqtOsccTimer->setSingleShot(true);
    QObject::connect(rqtOsccTimer, &QTimer::timeout, [=]() {
        ui_.EnableButton->setEnabled(true);
    });
    QMetaObject::invokeMethod(rqtOsccTimer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_oscc_cmd
PLUGINLIB_EXPORT_CLASS(rqt_oscc_cmd::MyPlugin, rqt_gui_cpp::Plugin)
