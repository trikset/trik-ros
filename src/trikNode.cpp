#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>
#include <QtGui/QApplication>

#include "ledCommand.h"

trikControl::BrickInterface *brick;
trikControl::LedInterface *led;
trikControl::SensorInterface *distanceSensor;
trikControl::MotorInterface *leftMotor;
trikControl::MotorInterface *rightMotor;

void ledCallback(const std_msgs::Int32 cmd) {
    switch (cmd.data) {
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

void cmdVelCallback(const geometry_msgs::Twist twist) {
    int power = int(twist.linear.x * 100);
    leftMotor->setPower(power);
    rightMotor->setPower(power);
}

int main(int argc, char **argv) {
    // start Qt server
    int qargc = 2;
    const char *qargv[] = {"standalone_trik_node", "-qws"};
    QApplication app(qargc, (char **) qargv);

    // init ROS node
    ros::init(argc, argv, "standalone_trik_node");
    ros::NodeHandle nh;
    ros::Rate loopRate(1);


    brick = trikControl::BrickFactory::create("/home/root/trik/system-config.xml",
                                              "/home/root/trik/model-config.xml",
                                              ".");
    led = brick->led();
    distanceSensor = brick->sensor("A1");
    leftMotor = brick->motor("M1");
    rightMotor = brick->motor("M3");

//    ros::Publisher distancePub = nh.advertise<std_msgs::Int32>("distance", 10);
    ros::Subscriber ledCmdSubscriber = nh.subscribe("cmd_led", 10, ledCallback, ros::TransportHints().udp());
    ros::Subscriber velCmdSubscriber = nh.subscribe("cmd_vel", 1, cmdVelCallback, ros::TransportHints().udp());


    while (ros::ok()) {
//        std_msgs::Int32 distanceMsg;
//        distanceMsg.data = distanceSensor->read();
//        distancePub.publish(distanceMsg);

//        ros::spinOnce();
//        loopRate.sleep();

        ros::spin();
    }

    return 0;
}
