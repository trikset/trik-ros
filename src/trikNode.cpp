#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <trikRuntime/trikControl/brickInterface.h>
#include <trikRuntime/trikControl/brickFactory.h>
#include <QtGui/QApplication>

#include "ledCommand.h"

trikControl::BrickInterface *brick;
trikControl::LedInterface *led;
trikControl::SensorInterface *distanceSensor;

void led_callback(const std_msgs::Int32 cmd) {
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

int main(int argc, char **argv) {
    // start Qt server
    int qargc = 2;
    const char *qargv[] = {"standalone_trik_node", "-qws"}; // todo: try QApplication::Tty for console app?
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

    ros::Publisher distancePub = nh.advertise<std_msgs::Int32>("distance", 10);
    ros::Subscriber sub = nh.subscribe("led_cmd", 10, led_callback);

    while (ros::ok()) {
        std_msgs::Int32 distanceMsg;
        distanceMsg.data = distanceSensor->read();
        distancePub.publish(distanceMsg);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
