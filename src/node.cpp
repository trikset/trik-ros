#include <ros/ros.h>
#include <QtGui/QApplication>
#include "trik.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trik");
    ros::NodeHandle nodeHandle;

    int qargc = 2;
    const char *qargv[] = {"trik", "-qws"};
    QApplication app(qargc, (char **) qargv);

    trikRos::Trik trik(&nodeHandle);

    return app.exec();
}
