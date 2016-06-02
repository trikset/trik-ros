//
// Created by Eugene on 29/05/16.
//


#include "motor.h"

using namespace trikRos;

Motor::Motor(trikControl::MotorInterface *device, ros::NodeHandle *nodeHandle, std::string &topic) {
    powerSubscriber_ = nodeHandle->subscribe(topic, 1, &Motor::powerCallback, this);
    device_ = device;
}

void Motor::setPower(int power) {
    device_->setPower(power);
}

void Motor::powerCallback(std_msgs::Int32 power) {
    setPower(power.data);
}
