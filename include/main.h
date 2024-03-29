/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

typedef struct ChassisVelocityPair {
    float left_v;
    float right_v;
} ChassisVelocityPair;

#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp"
#include "pros/api_legacy.h"


// constants and configuration
#include "data/constants.h"
#include "data/configuration.h"

// utilities
#include "utilities/coordinates.h"
#include "utilities/math.h"

// algorithms
#include "algorithms/odometry/odometry.h"
#include "algorithms/motion_profile/motion_profile.h"
#include "algorithms/catmull_rom/catmull_rom.h"

// subsystems
#include "subsystems/catapult/catapult.h"
#include "subsystems/vision/vision.h"
#include "subsystems/chassis/chassis.h"
#include "subsystems/intake/intake.h"
#include "subsystems/roller/roller.h"
#include "subsystems/optical/optical.h"
#include "subsystems/expansion/expansion.h"
#include "subsystems/blocker/blocker.h"
#include "../src/subsystems/graphical_interface/graphical_interface.h"

// autons
#include "autonomous/auton_first_scoring.h"
#include "autonomous/auton_first_support.h"
#include "autonomous/auton_second_scoring.h"
#include "autonomous/skill.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
struct Core {
	// controller
	okapi::Controller*   controller;
	okapi::Controller*   partner;
	// chassis
	okapi::Motor*        chassis_left_front;
	okapi::Motor*        chassis_left_middle;
	okapi::Motor*        chassis_left_back;
	okapi::Motor*        chassis_right_front;
	okapi::Motor*        chassis_right_middle;
	okapi::Motor*        chassis_right_back;
	// accessories
	okapi::Motor*        intake;
	okapi::Motor*        roller;
	okapi::Motor*        catapult_motor;
	pros::ADIDigitalOut* piston_booster;
	pros::ADIDigitalOut* expansion_left;
	pros::ADIDigitalOut* expansion_right;
	pros::ADIDigitalOut* blocker_left;
	pros::ADIDigitalOut* blocker_right;
	pros::ADIDigitalOut* blocker_top;

	// sensors
	okapi::ADIEncoder*   left_tracking_wheel;
	okapi::ADIEncoder*   right_tracking_wheel;
	okapi::ADIEncoder*   front_tracking_wheel;
	okapi::ADIEncoder*   back_tracking_wheel;
	okapi::ADIEncoder*   middle_tracking_wheel;
	pros::Imu*           imu_first;
	pros::Imu*           imu_second;
	pros::ADIDigitalIn*  catapult_load_sensor;
	pros::Vision*        vision_goal;
	
};


void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
#include <iostream>
#include <math.h>
#include <vector>

#endif

#endif  // _PROS_MAIN_H_
