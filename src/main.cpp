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
pros::ADIDigitalOut expansion_left        = pros::ADIDigitalOut(Configuration::Digital::EXPANSION_LEFT);
pros::ADIDigitalOut expansion_right       = pros::ADIDigitalOut(Configuration::Digital::EXPANSION_RIGHT);
pros::ADIDigitalOut blocker_left          = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_LEFT);
pros::ADIDigitalOut blocker_right         = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_RIGHT);
pros::ADIDigitalOut blocker_top           = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_TOP);

// sensors
okapi::ADIEncoder   middle_tracking_wheel = okapi::ADIEncoder({Configuration::EXPANDER, Configuration::Analog::ODOMETRY[0], Configuration::Analog::ODOMETRY[1]}, false);
pros::Imu           imu_first             = pros::Imu(Configuration::Analog::IMU[0]);
pros::Imu           imu_second            = pros::Imu(Configuration::Analog::IMU[1]);
pros::ADIDigitalIn  catapult_load_sensor  = pros::ADIDigitalIn({{Configuration::EXPANDER, Configuration::Digital::CATAPULT_LOAD_SENSOR}});
pros::Vision        vision_goal           = pros::Vision(Configuration::Digital::VISION_GOAL);

std::shared_ptr<Catapult> cata;
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
	core.expansion_left       = &expansion_left;
	core.expansion_right      = &expansion_right;
	core.catapult_motor       = &catapult;
	core.piston_booster       = &piston_booster;
	core.blocker_left         = &blocker_left;
	core.blocker_right        = &blocker_right;
	core.blocker_top          = &blocker_top;
	// sensors
	core.middle_tracking_wheel= &middle_tracking_wheel;
	core.imu_first            = &imu_first;
    core.imu_second           = &imu_second;
	core.catapult_load_sensor = &catapult_load_sensor;
	core.vision_goal          = &vision_goal;

	core.imu_first->reset(true);
	core.imu_second->reset(true);
	cata = std::make_shared<Catapult>(&core);
	GraphicalInterface interface;
	printf("Interface started\n");
	pros::delay(1000);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// cata.reset();
	// odometry.reset();
	// intake_obj.reset();
	// roller_obj.reset();
}

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

	GraphicalInterface::InterfaceSelector position = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_POSITION);
	GraphicalInterface::InterfaceSelector game_team = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_TEAM);
	GraphicalInterface::InterfaceSelector mode = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_MODE);
	GraphicalInterface::InterfaceSelector game_round = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_ROUND);
	
	if (game_round == GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_SKILL) {
		Skill auton = Skill(&core, cata, false);
		auton.run();
		return;
	}

	// team
	bool position_offset = game_team == GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED;
	
	// fist position scoring mode
	if      (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE) {
		AutonFirstScoring auton = AutonFirstScoring(&core, cata, position_offset);
		auton.run();
		printf("#1 score, offset %d\n", position_offset);
	}
	// second position scoring mode
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE) {
		AutonSecondScoring auton = AutonSecondScoring(&core, cata, position_offset);
		auton.run();
		printf("#2 score, offset %d\n", position_offset);
	}
	// fist position support mode (WP)
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
		     mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT) {
		AutonFirstSupport auton = AutonFirstSupport(&core, cata, position_offset);
		auton.run();
		printf("#1 support, offset %d\n", position_offset);
	}
	// idle mode
	else if (mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE) {
		printf("idle\n");
		;
	}
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
    Chassis chassis = Chassis(&core);
    Intake intake = Intake(&core);
    Roller roller = Roller(&core);
	Expansion expansion = Expansion(&core);


	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	printf("finish setbreakmode\n");


	bool is_blocking_left = false;
	bool is_blocking_right = false;
	bool is_blocking_top = false;

	bool blocker_button_state_left = false;
	bool blocker_button_state_right = false;
	
	cata->set_boost(false);
	cata->set_voltage(9000);
	

	while (true) {
		// locomotion
		if (core.controller->getDigital(Configuration::Controls::AUTOAIM_BUTTON)) {
			chassis.auto_aim();
		} else {
			chassis.cheezyDrive(core.controller->getAnalog(Configuration::Controls::FORWARD_AXIS), 0.75*core.controller->getAnalog(Configuration::Controls::TURN_AXIS));
		}
		
		// intake & roller
		if (core.controller->getDigital(Configuration::Controls::INTAKE_BUTTON)) {
			intake.turn_on();
		} else if (core.controller->getDigital(Configuration::Controls::INTAKE_REV_BUTTON)) {
			intake.turn_on_rev();
		} else {
			intake.turn_off();
		}

		// shoot
		if (core.controller->getDigital(Configuration::Controls::SHOOT_BUTTON)) {
			cata->fire();
		}

		/*
		Endgame
		*/

		// left blocker
		if (!blocker_button_state_left && core.controller->getDigital(Configuration::Controls::BLOCKER_LEFT_BUTTON) && !is_blocking_left) {
			is_blocking_left = true;
			blocker_button_state_left = true;
			core.blocker_left->set_value(true);
		} else if (!blocker_button_state_left && core.controller->getDigital(Configuration::Controls::BLOCKER_LEFT_BUTTON) && is_blocking_left) {
			is_blocking_left = false;
			blocker_button_state_left = true;
			core.blocker_left->set_value(false);
		} else if (!core.controller->getDigital(Configuration::Controls::BLOCKER_LEFT_BUTTON)) {
			blocker_button_state_left = false;
		}

		// right blocker
		if (!blocker_button_state_right && core.controller->getDigital(Configuration::Controls::BLOCKER_RIGHT_BUTTON) && !is_blocking_right) {
			is_blocking_right = true;
			blocker_button_state_right = true;
			core.blocker_right->set_value(true);
		} else if (!blocker_button_state_right && core.controller->getDigital(Configuration::Controls::BLOCKER_RIGHT_BUTTON) && is_blocking_right) {
			is_blocking_right = false;
			blocker_button_state_right = true;
			core.blocker_right->set_value(false);
		} else if (!core.controller->getDigital(Configuration::Controls::BLOCKER_RIGHT_BUTTON)) {
			blocker_button_state_right = false;
		}

		// top blocker
		if (core.controller->getDigital(Configuration::Controls::BLOCKER_TOP_BUTTON_MASTER)) {
			is_blocking_top = true;
			core.blocker_top->set_value(true);
		}

		if (core.partner->getDigital(Configuration::Controls::SAFETY_BUTTON)) {
			core.partner->setText(0,0,"SAFE OFF");
			if (core.partner->getDigital(Configuration::Controls::BLOCKER_TOP_BUTTON_PARTNER)) {
				is_blocking_top = true;
				core.blocker_top->set_value(true);
			}
		} else {
			core.partner->setText(0,0,"SAFE ON ");
		}

		// expansion
		if (core.controller->getDigital(Configuration::Controls::EXPANSION_LEFT_BUTTON)) {
			core.expansion_left->set_value(true);
		}
		if (core.controller->getDigital(Configuration::Controls::EXPANSION_RIGHT_BUTTON)) {
			core.expansion_right->set_value(true);
		}
		if (core.controller->getDigital(Configuration::Controls::EXPANSION_ALL_BUTTON)) {
			core.expansion_left->set_value(true);
			core.expansion_right->set_value(true);
		}


		pros::delay(20);
	}
}
