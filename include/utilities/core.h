#pragma once
#include "main.h"

namespace Core {
    namespace Controllers {
        okapi::Controller*   controller;
    }
    namespace Motors {
        // chassis
        okapi::Motor*        chassis_left_front;
	    okapi::Motor*        chassis_left_middle;
	    okapi::Motor*        chassis_left_back;
	    okapi::Motor*        chassis_right_first;
	    okapi::Motor*        chassis_right_middle;
	    okapi::Motor*        chassis_right_back;
        // accessories
	    okapi::Motor*        intake;
	    okapi::Motor*        roller;
    }
    namespace Pneumatics {
        // expansion
        pros::ADIDigitalOut* expansion;
    }
    namespace Sensors {
        okapi::ADIEncoder*   odometry_wheel;
	    pros::Imu*           inertial;
    }
}