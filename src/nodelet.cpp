// Eugene Auduchinok

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>
#include <QtGui/qapplication.h>

#include "ledCommand.h"

class TrikNodelet : public nodelet::Nodelet {
public:
    void onInit() {
        // start Qt server
        int qargc = 2;
        app = new QApplication(qargc, (char **) qargv);

        brick = trikControl::BrickFactory::create("/home/root/trik/system-config.xml",
                                                  "/home/root/trik/model-config.xml",
                                                  ".");
        led = brick->led();
        led->off();
        distanceSensor = brick->sensor("A1");

        ros::NodeHandle nh = getMTNodeHandle();
        ros::Subscriber ledCmdSub = nh.subscribe("led_cmd", 10, &TrikNodelet::ledCmdCallback, this);
        led->red();
        timer = nh.createTimer(1, &TrikNodelet::timerCallback, this);
        distancePub = nh.advertise<std_msgs::Int32>("distance", 10);
        led->green();


// old way
//        ros::Subscriber sub = nh.subscribe<std_msgs::Int32>("led_cmd", 10,
//                                                            boost::bind(&TrikNodelet::ledCmdCallback, this, _1));
//        timer = nh.createTimer(ros::Duration(1.0), boost::bind(&TrikNodelet::rangeSensorsCallback, this, _1));
    }

private:
    const char *qargv[2] = {"standalone_trik_node", "-qws"};
    QApplication *app;
    ros::Timer timer;
    ros::Publisher distancePub;
    trikControl::BrickInterface *brick;
    trikControl::LedInterface *led;
    trikControl::SensorInterface *distanceSensor;


    void timerCallback(const ros::TimerEvent &event) {
        std_msgs::Int32Ptr distanceMsg(new std_msgs::Int32);
        distanceMsg->data = distanceSensor->read();
        distancePub.publish(distanceMsg);
    }

    void ledCmdCallback(const std_msgs::Int32ConstPtr &cmd) {
        switch (cmd->data) {
            case LedCommand::OFF:
                led->off();
                break;
            case LedCommand::GREEN:
                led->green();
                break;
            case LedCommand::RED:
                led->red();
                break;
            case LedCommand::ORANGE:
                led->orange();
                break;
            default:
                break;
        }
    }
};

PLUGINLIB_EXPORT_CLASS(TrikNodelet, nodelet::Nodelet)
