#include "ros/ros.h"
#include "trikControl/brickInterface.h"
#include "trikControl/brickFactory.h"
#include "QtGui/QApplication"

int main(int argc, char **argv) {
    ros::init(argc, argv, "standalone_trik_node");

//    int qargc = 2;
//    const char *qargv[] = {"standalone_trik_node", "-qws"}; // todo: try QApplication::Tty for console app?
//    QApplication app(qargc, (char **) qargv);
    QApplication app(argc, argv);

    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    trikControl::BrickInterface *brick = trikControl::BrickFactory::create("/etc/trik/trikRuntime/system-config.xml",
                                                                           "/etc/trik/trikRuntime/model-config.xml",
                                                                           ".");
    trikControl::LedInterface *led = brick->led();
    trikControl::SensorInterface *distance = brick->sensor("A1");

    for (int i = 0; ros::ok(); i++) {
        if (distance->read() < 20) {
            if (i % 2 == 0) {
                led->green();
            } else {
                led->red();
            }
        } else {
            led->off();
        }

        ros::spinOnce(); // allow callbacks
        loop_rate.sleep();
    }
    return 0;
}

