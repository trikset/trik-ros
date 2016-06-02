//
// Created by Eugene on 28/05/16.
//

#include "sensor.h"

Sensor::Sensor(trikControl::SensorInterface *device, ros::NodeHandle *nodeHandle, std::string &topic)
        : publisher_(nodeHandle->advertise<sensor_msgs::Range>(topic, 10)) {

    device_ = device;
    minRange_ = device_->minValue();
    maxRange_ = device_->maxValue();
}

void Sensor::publish() {
    int distance = device_->read();

    sensor_msgs::Range distanceMsg;
    distanceMsg.range = distance * 0.01f;
    distanceMsg.min_range = minRange_;
    distanceMsg.max_range = maxRange_;

    publisher_.publish(distanceMsg);
}
