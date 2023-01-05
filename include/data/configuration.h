#include "main.h"

using namespace okapi;

namespace Configuration {
    namespace Motors {
        // chassis
        inline int CHASSIS_LEFT_FRONT   = -1;
        inline int CHASSIS_LEFT_MIDDLE  = 2;
        inline int CHASSIS_LEFT_BACK    = -3;
        inline int CHASSIS_RIGHT_FRONT  = 4;
        inline int CHASSIS_RIGHT_MIDDLE = -5;
        inline int CHASSIS_RIGHT_BACK   = 6;
        // accessories
        inline int INTAKE = 7;
        inline int ROLLER = 7;
        inline int CATAPULT = 8;
    };

    namespace Analog {
        inline std::uint8_t     EXPANSION             = ' ';
        inline std::uint8_t     ODOMETRY[2]           = {'A', 'B'};
        inline std::uint8_t     IMU[2]                = {7, 8};
    };

    namespace Digital {
        inline std::uint8_t     CATAPULT_LOAD_SENSOR = 'C';
        inline std::uint8_t     VISION_GOAL          = 1;
    }

    namespace Controls {
        inline ControllerAnalog  FORWARD_AXIS      = ControllerAnalog::leftY;
        inline ControllerAnalog  TURN_AXIS         = ControllerAnalog::rightX;

        inline ControllerDigital INTAKE_BUTTON     = ControllerDigital::R1;
        inline ControllerDigital INTAKE_BUTTON_REV = ControllerDigital::A;
        inline ControllerDigital SHOOT_BUTTON      = ControllerDigital::L2;
        inline ControllerDigital ROLL_BUTTON       = ControllerDigital::L2;
        inline ControllerDigital EXPANSION_BUTTON  = ControllerDigital::up;
        inline ControllerDigital BLOCKER_BUTTON    = ControllerDigital::L2;
    };
};
