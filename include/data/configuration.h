#include "main.h"

using namespace okapi;

namespace CONFIG {
    namespace MOTOR_PORTS {
        namespace CHASSIS_MOTORS {
            inline int LFMOTOR = 1;
            inline int LMMOTOR = 2;
            inline int LBMOTOR = 3;
            inline int RFMOTOR = -4;
            inline int RMMOTOR = -5;
            inline int RBMOTOR = -6;
        };
        inline int INTAKE_MOTOR = 7;
        inline int ROLLER_MOTOR = 7;

    };

    namespace Controls {
        inline ControllerAnalog FORWARD_AXIS = ControllerAnalog::leftY;
        inline ControllerAnalog TURN_AXIS = ControllerAnalog::rightX;

        inline ControllerDigital INTAKE_BUTTON = ControllerDigital::R1;
        inline ControllerDigital INTAKE_BUTTON_REV = ControllerDigital::A;
        inline ControllerDigital SHOOT_BUTTON = ControllerDigital::L2;
        inline ControllerDigital ROLL_BUTTON = ControllerDigital::L2;
        inline ControllerDigital EXPANSION_BUTTON = ControllerDigital::up;
        inline ControllerDigital BLOCKER_BUTTON = ControllerDigital::L2;
    };

    namespace ROBOT_PARAMETERS {
        inline QLength WHEEL_DIAMETER = 2.75_in;
        inline QLength TRACK_LENGTH = 27_cm;
        inline QLength MIDDLE_ENCODER_DISTANCE = 12_cm;
        inline QLength TRACKING_WHEEL_DIAMETER = 2.75_in;
        inline float EXTERNAL_GEAR_RATIO = 1;
        inline AbstractMotor::gearset MOTOR_GEARSET = AbstractMotor::gearset::blue;
    }
};
