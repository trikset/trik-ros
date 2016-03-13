// Eugene Auduchinok

#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Int32.h"

#include "trikControl/brickInterface.h"
#include "trikControl/brickFactory.h"
#include <QtGui/qapplication.h>

#include "led_command.h"

class TrikNodelet : public nodelet::Nodelet {
public:
    void onInit() {
        // start Qt server
        int qargc = 2;
        const char *qargv[] = {"standalone_trik_node", "-qws"}; // todo: try QApplication::Tty for console app?
        QApplication app(qargc, (char **) qargv);

        NODELET_INFO("Initializing TrikNodelet...");
        brick = trikControl::BrickFactory::create();
    }

private:
    trikControl::BrickInterface *brick;
    trikControl::LedInterface *led;
    trikControl::SensorInterface *sensor;

    void led_callback(const std_msgs::Int32 cmd) {
        switch (cmd.data) {
            case LedCommand::OFF:
                led->off();
                break;
            case LedCommand::GREEN:
                led->green();
                break;
            case LedCommand::RED:
                led->red();
                break;
            case LedCommand::ORANGE:
                led->orange();
                break;
            default:
                break;
        }
    }

};

PLUGINLIB_EXPORT_CLASS(TrikNodelet, nodelet::Nodelet)
