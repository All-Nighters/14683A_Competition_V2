#include <vector>
#include "main.h"


struct Core core;

// controller
okapi::Controller   controller            = okapi::Controller(okapi::ControllerId::master);
okapi::Controller   partner               = okapi::Controller(okapi::ControllerId::partner);
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
pros::ADIDigitalOut piston_booster        = pros::ADIDigitalOut({{Configuration::EXPANDER, Configuration::Digital::BOOSTER}});
pros::ADIDigitalOut expansion             = pros::ADIDigitalOut(Configuration::Digital::EXPANSION);
// sensors
okapi::ADIEncoder   middle_tracking_wheel = okapi::ADIEncoder({Configuration::EXPANDER, Configuration::Analog::ODOMETRY[0], Configuration::Analog::ODOMETRY[1]}, false);
pros::Imu           imu_first             = pros::Imu(Configuration::Analog::IMU[0]);
pros::Imu           imu_second            = pros::Imu(Configuration::Analog::IMU[1]);
pros::ADIDigitalIn  catapult_load_sensor  = pros::ADIDigitalIn({{Configuration::EXPANDER, Configuration::Digital::CATAPULT_LOAD_SENSOR}});
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
	core.partner              = &partner;
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
	core.piston_booster       = &piston_booster;
	// sensors
	core.middle_tracking_wheel= &middle_tracking_wheel;
	core.imu_first            = &imu_first;
    core.imu_second           = &imu_second;
	core.catapult_load_sensor = &catapult_load_sensor;
	core.vision_goal          = &vision_goal;

	core.imu_first->reset(true);
	core.imu_second->reset(true);
	GraphicalInterface interface;
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
void autonomous() {
	Odom odometry = Odom(&core, OdomMode::MIDDLETW_IMU);
	Chassis chassis = Chassis(&core, &odometry);
	Catapult cata = Catapult(&core);
	Intake intake = Intake(&core);

	GraphicalInterface::InterfaceSelector position = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_POSITION);
	GraphicalInterface::InterfaceSelector game_team = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_TEAM);
	GraphicalInterface::InterfaceSelector mode = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_MODE);
	GraphicalInterface::InterfaceSelector game_round = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_ROUND);
	
	if (game_round == GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_SKILL) {

		return;
	}

	// team
	bool position_offset = game_team == GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED;
	
	// fist position scoring mode
	if      (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE) {
		AutonFirstScoring auton = AutonFirstScoring(&chassis, &cata, &intake, position_offset);
		auton.run();
	}
	// second position scoring mode
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE) {
		AutonSecondScoring auton = AutonSecondScoring(&chassis, &cata, &intake, position_offset);
		auton.run();
	}
	// fist position support mode (WP)
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT) {
		AutonFirstSupport auton = AutonFirstSupport(&chassis, &cata, &intake, position_offset);
		auton.run();
	}
	// idle mode
	else if (mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE) {
		;
	}
	// bool position_offset = false;
	// AutonFirstScoring auton = AutonFirstScoring(&chassis, &cata, &intake, position_offset);
	// auton.run();
}

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
	// Catapult cata = Catapult(&core);
	Odom odometry = Odom(&core, OdomMode::MIDDLETW_IMU);
	Intake intake = Intake(&core);
	Chassis chassis = Chassis(&core, &odometry);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	std::vector<Coordinates> pathway;
	bool is_boosting = false;
	// odometry.setState(0, 0, 0);
	// for (int i = 0; i < 10; i++) {
	// 	pathway.push_back(Coordinates(6*i, 0, 0));
	// }
	// for (int i = 0; i < 10; i++) {
	// 	pathway.push_back(Coordinates(60, 3*i, 0));
	// }
	// chassis.followPath(pathway, true);
	while (true) {
		// printf("%f\n", chassis.getLeftPosition());
		chassis.cheezyDrive(core.controller->getAnalog(Configuration::Controls::FORWARD_AXIS), core.controller->getAnalog(Configuration::Controls::TURN_AXIS));
		if (core.controller->getDigital(Configuration::Controls::INTAKE_BUTTON)) {
			intake.turn_on();
		} else {
			intake.turn_off();
		}
		// if (core.controller->getDigital(Configuration::Controls::SHOOT_BUTTON)) {
		// 	cata.fire();
		// }
		// printf("%f\n", odometry.getState().theta);
		pros::delay(20);
	}
}
