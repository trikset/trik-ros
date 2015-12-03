#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

class TrikNode : public nodelet::Nodelet {
public:
  void onInit() {
    NODELET_DEBUG("Initializing TrikNode...");
  }
};

PLUGINLIB_EXPORT_CLASS(TrikNode, nodelet::Nodelet)
