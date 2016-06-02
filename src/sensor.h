//
// Created by Eugene Auduchinok on 28/05/16.
//

#ifndef TRIK_ROS_RANGESENSOR_H
#define TRIK_ROS_RANGESENSOR_H

#include <ros/ros.h>
#include <trikControl/brickInterface.h>
#include <sensor_msgs/Range.h>

class Sensor {
public:
    Sensor(trikControl::SensorInterface *device, ros::NodeHandle *nodeHandle, std::string &topic);

    void publish();

private:
    ros::Publisher publisher_;
    trikControl::SensorInterface *device_;
    float minRange_;
    float maxRange_;
};


#endif //TRIK_ROS_RANGESENSOR_H
