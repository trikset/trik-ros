//
// Created by Eugene Auduchinok on 28/05/16.
//

#ifndef TRIK_ROS_TRIK_H
#define TRIK_ROS_TRIK_H

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <trikControl/brickInterface.h>

#include "sensor.h"
#include "motor.h"

namespace trikRos {
    class Trik {
    public:
        Trik(ros::NodeHandle *nodeHandle);

        void publishSensorsData(const ros::TimerEvent &event);

        void ledCallback(const std_msgs::Int32 cmd);

        void velocityCallback(const geometry_msgs::Twist twist);

    private:
        trikControl::BrickInterface *brick_;
        trikControl::LedInterface *led_;

        QVector<Sensor *> *sensors_;

        QVector<Motor *> *powerMotors_;
        QVector<QString> *leftPowerMotorPorts_;
        QVector<QString> *rightPowerMotorPorts_;
        QVector<Motor *> *leftPowerMotors_;
        QVector<Motor *> *rightPowerMotors_;

        ros::NodeHandle *nodeHandle_;
        ros::Timer sensorsTimer_;
        ros::Subscriber ledSubscriber_;
        ros::Subscriber velocitySubscriber_;

        bool checkPortEnabled(QString port);
        void setMotorsPower(QVector<Motor *> *motors, int power);
        QVector<Sensor *> *initializeSensors();
        QVector<Motor *> *initializeMotors(trikControl::MotorInterface::Type type, std::string topicTemplate);
    };
}


#endif //TRIK_ROS_TRIK_H
