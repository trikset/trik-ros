//
// Created by Eugene Auduchinok on 29/05/16.
//

#ifndef TRIK_ROS_MOTOR_H
#define TRIK_ROS_MOTOR_H

#include <trikControl/motorInterface.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace trikRos {
    class Motor {
    public:
        Motor(trikControl::MotorInterface *device, ros::NodeHandle *nodeHandle, std::string &topic);

        void setPower(int power);

        void powerCallback(std_msgs::Int32 power);

    private:
        trikControl::MotorInterface *device_;
        ros::Subscriber powerSubscriber_;

    };
}


#endif //TRIK_ROS_MOTOR_H
