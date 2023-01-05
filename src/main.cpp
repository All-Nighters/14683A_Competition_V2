#include <vector>
#include "main.h"


struct Core core;

// controller
okapi::Controller   controller            = okapi::Controller();
// chassis
okapi::Motor        chassis_left_front    = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_FRONT);
okapi::Motor        chassis_left_middle   = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_MIDDLE);
okapi::Motor        chassis_left_back     = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_BACK);
okapi::Motor        chassis_right_front   = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_FRONT);
okapi::Motor        chassis_right_middle  = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_MIDDLE);
okapi::Motor        chassis_right_back    = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_BACK);
// accessories
okapi::Motor        intake                = okapi::Motor(Configuration::Motors::INTAKE);
okapi::Motor        roller                = okapi::Motor(Configuration::Motors::ROLLER);
okapi::Motor        catapult              = okapi::Motor(Configuration::Motors::CATAPULT);
pros::ADIDigitalOut expansion             = pros::ADIDigitalOut(Configuration::Analog::EXPANSION);
// sensors
okapi::ADIEncoder   middle_tracking_wheel = okapi::ADIEncoder(Configuration::Analog::ODOMETRY[0], Configuration::Analog::ODOMETRY[1], false);
pros::Imu           imu_first             = pros::Imu(Configuration::Analog::IMU[0]);
pros::Imu           imu_second            = pros::Imu(Configuration::Analog::IMU[1]);
pros::ADIDigitalIn  catapult_load_sensor  = pros::ADIDigitalIn(Configuration::Digital::CATAPULT_LOAD_SENSOR);
pros::Vision        vision_goal           = pros::Vision(Configuration::Digital::VISION_GOAL);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// controller
	core.controller           = &controller;
	// chassis
	core.chassis_left_front   = &chassis_left_front;
	core.chassis_left_middle  = &chassis_left_middle;
	core.chassis_left_back    = &chassis_left_back;
	core.chassis_right_front  = &chassis_right_front;
	core.chassis_right_middle = &chassis_right_middle;
	core.chassis_right_back   = &chassis_right_back;
	// accessories
	core.intake               = &intake;
	core.roller               = &roller;
	core.expansion            = &expansion;
	core.catapult_motor       = &catapult;
	// sensors
	core.middle_tracking_wheel= &middle_tracking_wheel;
	core.imu_first            = &imu_first;
    core.imu_second           = &imu_second;
	core.catapult_load_sensor = &catapult_load_sensor;
	core.vision_goal          = &vision_goal;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // Odom odometry = Odom(&core, OdomMode::MIDDLETW_IMU);
    Chassis chassis = Chassis(&core);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	// Catapult cata = Catapult(&core);
	while (true) {
		chassis.cheezyDrive(core.controller->getAnalog(okapi::ControllerAnalog::leftY), core.controller->getAnalog(okapi::ControllerAnalog::rightX));
		// if (core.controller->getDigital(Configuration::Controls::SHOOT_BUTTON)) {
		// 	cata.fire();
		// 	cata.wait_until_reloaded();
		// }
		pros::delay(10);
	}
}
