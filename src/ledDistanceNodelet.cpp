// Eugene Auduchinok

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ledCommand.h"

class LedDistance : public nodelet::Nodelet {
public:
    void onInit() {
        ros::NodeHandle nh = getMTNodeHandle();
        ros::Subscriber distanceSub = nh.subscribe("distance", 10, &LedDistance::distanceCallback, this);
        ledPub = nh.advertise<std_msgs::Int32>("led_cmd", 10);
    }

private:
    ros::Publisher ledPub;

    void distanceCallback(const std_msgs::Int32ConstPtr &distMsg) {
        std_msgs::Int32Ptr ledMsg(new std_msgs::Int32);

        int dist = distMsg->data;
        if (dist < 20) {
            ledMsg->data = dist % 2 == 0 ? LedCommand::GREEN : LedCommand::RED;
        } else {
            ledMsg->data = LedCommand::OFF;
        }

        ledPub.publish(ledMsg);
    }
};

PLUGINLIB_EXPORT_CLASS(LedDistance, nodelet::Nodelet)
