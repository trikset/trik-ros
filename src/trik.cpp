//
// Created by Eugene Auduchinok on 28/05/16.
//

#include <ros/ros.h>
#include <trikControl/brickFactory.h>
#include <trikControl/motorInterface.h>

#include "trik.h"
#include "ledCommand.h"

using namespace trikRos;
using namespace trikControl;


Trik::Trik(ros::NodeHandle *nodeHandle) {
    brick_ = BrickFactory::create("/home/root/trik/", "/home/root/trik/media/");

    sensors_ = initializeSensors();

    leftPowerMotorPorts_ = new QVector<QString>({"M1", "M3"});
    rightPowerMotorPorts_ = new QVector<QString>({"M2", "M4"});

    leftPowerMotors_ = new QVector<Motor *>;
    rightPowerMotors_ = new QVector<Motor *>;

    powerMotors_ = initializeMotors(MotorInterface::Type::powerMotor, "power_motor");
    powerMotors_ = initializeMotors(MotorInterface::Type::servoMotor, "servo_motor");


    // todo: try using ros::TransportHints().udp() for subscribers
    auto rate = ros::Rate(nodeHandle->param("rate", 20));
    sensorsTimer_ = nodeHandle->createTimer(ros::Duration(rate), &Trik::publishSensorsData, this);
    ledSubscriber_ = nodeHandle->subscribe("cmd_led", 1, &Trik::ledCallback, this);
    velocitySubscriber_ = nodeHandle->subscribe("cmd_vel", 10, &Trik::velocityCallback, this);
    nodeHandle_ = nodeHandle;
}


QVector<Sensor *> *Trik::initializeSensors() {
    QVector<QPair<QString, std::string>> sensorsPortsTopics = {{"A1", "ir1"},
                                                                   {"A2", "ir2"},
                                                                   {"A5", "light1"},
                                                                   {"A6", "light2"},
                                                                   {"D1", "ultrasonic1"},
                                                                   {"D2", "ultrasonic2"}};

    auto sensors = new QVector<Sensor *>();
    for (auto sensor : sensorsPortsTopics) {
        if (!checkPortEnabled(sensor.first)) continue;

        auto topic = nodeHandle_->param(sensor.first.toStdString() + "_topic", "sensors/" + sensor.second);
        sensors->push_back(new Sensor(brick_->sensor(sensor.first), nodeHandle_, topic));
    }
    return sensors;
}


QVector<Motor *> *Trik::initializeMotors(MotorInterface::Type type, std::string topicTemplate) {
    auto motors = new QVector<Motor *>();

    for (auto port : brick_->motorPorts(type)) {
        if (!checkPortEnabled(port)) continue;

        auto topic = topicTemplate + std::to_string(motors->size());
        Motor *motor = new Motor(brick_->motor(port), nodeHandle_, topic);
        motors->push_back(motor);

        if (type == MotorInterface::Type::powerMotor) {
            if (leftPowerMotorPorts_->contains(port)) leftPowerMotors_->push_back(motor);
            if (rightPowerMotorPorts_->contains(port)) rightPowerMotors_->push_back(motor);
        }
    }
    return motors;
}


bool Trik::checkPortEnabled(QString port) {
    return nodeHandle_->param(port.toStdString() + "_enabled", false);
}


void Trik::publishSensorsData(const ros::TimerEvent &event) {
    for (auto sensor : *sensors_) {
        sensor->publish();
    }
}


void Trik::ledCallback(const std_msgs::Int32 led) { // todo: define custom message type
    switch (led.data) {
        case LedCommand::OFF:
            led_->off();
            break;
        case LedCommand::GREEN:
            led_->green();
            break;
        case LedCommand::RED:
            led_->red();
            break;
        case LedCommand::ORANGE:
            led_->orange();
            break;
        default:
            break;
    }

}


void Trik::velocityCallback(const geometry_msgs::Twist twist) {
    int leftPower = int((twist.linear.x - twist.angular.z) * 100);
    int rightPower = int((twist.linear.x + twist.angular.z) * 100);

    setMotorsPower(leftPowerMotors_, leftPower);
    setMotorsPower(rightPowerMotors_, rightPower);
}


void Trik::setMotorsPower(QVector<Motor *> *motors, int power) {
    for (auto motor : *motors) {
        motor->setPower(power);
    }
}
