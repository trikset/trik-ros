#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "trikControl/brickInterface.h"
#include "trikControl/brickFactory.h"

class TrikNode : public nodelet::Nodelet {
public:
  void onInit() {
    NODELET_DEBUG("Initializing TrikNode...");
    brick = trikControl::BrickFactory::create();
  }

private:
    trikControl::BrickInterface *brick;
};

PLUGINLIB_EXPORT_CLASS(TrikNode, nodelet::Nodelet)
