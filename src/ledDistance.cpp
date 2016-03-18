// Eugene Auduchinok, 03/13/2016

#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include "ledCommand.h"

ros::Publisher ledPub;

void distance_callback(const std_msgs::Int32 distMsg) {
    std_msgs::Int32 ledMsg;

    int dist = distMsg.data;
    if (dist < 20) {
        ledMsg.data = dist % 2 == 0 ? LedCommand::GREEN : LedCommand::RED;
    } else {
        ledMsg.data = LedCommand::OFF;
    }

    ledPub.publish(ledMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "led_distance");
    ros::NodeHandle nh;

    ledPub = nh.advertise<std_msgs::Int32>("led_cmd", 10);
    ros::Subscriber distSub = nh.subscribe("distance", 10, distance_callback);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
