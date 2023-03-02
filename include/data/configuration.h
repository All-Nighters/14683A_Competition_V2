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

    inline int EXPANDER = 10;
    namespace Analog {
        inline std::uint8_t     ODOMETRY[2]           = {'C', 'D'};
        inline std::uint8_t     IMU[2]                = {12, 14};
    };

    namespace Digital {
        inline std::uint8_t     CATAPULT_LOAD_SENSOR = 'H';
        inline std::uint8_t     BOOSTER              = 'F';
        inline std::uint8_t     EXPANSION_LEFT       = 'C';
        inline std::uint8_t     EXPANSION_RIGHT      = 'A';
        inline std::uint8_t     BLOCKER_RIGHT        = 'E';
        inline std::uint8_t     BLOCKER_LEFT         = 'B';
        inline std::uint8_t     BLOCKER_TOP          = 'D';

        inline std::uint8_t     VISION_GOAL          = 13;
    }

    namespace Controls {
        inline ControllerAnalog  FORWARD_AXIS                = ControllerAnalog::leftY;
        inline ControllerAnalog  TURN_AXIS                   = ControllerAnalog::rightX;

        inline ControllerDigital INTAKE_BUTTON               = ControllerDigital::R1;
        inline ControllerDigital INTAKE_REV_BUTTON           = ControllerDigital::R2;

        inline ControllerDigital AUTOAIM_BUTTON              = ControllerDigital::B;
        inline ControllerDigital SHOOT_BUTTON                = ControllerDigital::L2;
        inline ControllerDigital SAFETY_BUTTON               = ControllerDigital::left;
        inline ControllerDigital EXPANSION_LEFT_BUTTON       = ControllerDigital::left;
        inline ControllerDigital EXPANSION_RIGHT_BUTTON      = ControllerDigital::A;
        inline ControllerDigital BLOCKER_TOP_BUTTON_PARTNER  = ControllerDigital::X;
        inline ControllerDigital EXPANSION_ALL_BUTTON        = ControllerDigital::up;
        inline ControllerDigital BLOCKER_LEFT_BUTTON         = ControllerDigital::right;
        inline ControllerDigital BLOCKER_RIGHT_BUTTON        = ControllerDigital::Y;
        inline ControllerDigital BLOCKER_TOP_BUTTON_MASTER   = ControllerDigital::X;

    };
};
