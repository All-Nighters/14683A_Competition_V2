#include <vector>
#include "main.h"

struct Core core;

// controller
okapi::Controller controller = okapi::Controller(okapi::ControllerId::master);
okapi::Controller partner = okapi::Controller(okapi::ControllerId::partner);

// chassis
okapi::Motor chassis_left_front = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_FRONT);
okapi::Motor chassis_left_middle = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_MIDDLE);
okapi::Motor chassis_left_back = okapi::Motor(Configuration::Motors::CHASSIS_LEFT_BACK);
okapi::Motor chassis_right_front = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_FRONT);
okapi::Motor chassis_right_middle = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_MIDDLE);
okapi::Motor chassis_right_back = okapi::Motor(Configuration::Motors::CHASSIS_RIGHT_BACK);
// accessories
okapi::Motor intake = okapi::Motor(Configuration::Motors::INTAKE);
okapi::Motor roller = okapi::Motor(Configuration::Motors::ROLLER);
okapi::Motor catapult = okapi::Motor(Configuration::Motors::CATAPULT);
pros::ADIDigitalOut piston_booster = pros::ADIDigitalOut({{Configuration::EXPANDER, Configuration::Digital::BOOSTER}});
pros::ADIDigitalOut expansion_left = pros::ADIDigitalOut(Configuration::Digital::EXPANSION_LEFT);
pros::ADIDigitalOut expansion_right = pros::ADIDigitalOut(Configuration::Digital::EXPANSION_RIGHT);
pros::ADIDigitalOut blocker_left = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_LEFT);
pros::ADIDigitalOut blocker_right = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_RIGHT);
pros::ADIDigitalOut blocker_top = pros::ADIDigitalOut(Configuration::Digital::BLOCKER_TOP);

// sensors
okapi::ADIEncoder middle_tracking_wheel = okapi::ADIEncoder({Configuration::EXPANDER, Configuration::Analog::ODOMETRY[0], Configuration::Analog::ODOMETRY[1]}, false);
pros::Imu imu_first = pros::Imu(Configuration::Analog::IMU[0]);
pros::Imu imu_second = pros::Imu(Configuration::Analog::IMU[1]);
pros::ADIDigitalIn catapult_load_sensor = pros::ADIDigitalIn({{Configuration::EXPANDER, Configuration::Digital::CATAPULT_LOAD_SENSOR}});
pros::Vision vision_goal = pros::Vision(Configuration::Digital::VISION_GOAL);

std::shared_ptr<Catapult> cata;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	// controller
	core.controller = &controller;
	core.partner = &partner;
	// chassis
	core.chassis_left_front = &chassis_left_front;
	core.chassis_left_middle = &chassis_left_middle;
	core.chassis_left_back = &chassis_left_back;
	core.chassis_right_front = &chassis_right_front;
	core.chassis_right_middle = &chassis_right_middle;
	core.chassis_right_back = &chassis_right_back;
	// accessories
	core.intake = &intake;
	core.roller = &roller;
	core.expansion_left = &expansion_left;
	core.expansion_right = &expansion_right;
	core.catapult_motor = &catapult;
	core.piston_booster = &piston_booster;
	core.blocker_left = &blocker_left;
	core.blocker_right = &blocker_right;
	core.blocker_top = &blocker_top;
	// sensors
	core.middle_tracking_wheel = &middle_tracking_wheel;
	core.imu_first = &imu_first;
	core.imu_second = &imu_second;
	core.catapult_load_sensor = &catapult_load_sensor;
	core.vision_goal = &vision_goal;

	core.imu_first->reset(true);
	core.imu_second->reset(true);
	// cata = std::make_shared<Catapult>(&core);
	GraphicalInterface interface;
	printf("[Main]: Interface started\n");
	pros::delay(1000);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
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
void autonomous()
{
	printf("[Main]: Autonomous started\n");
	GraphicalInterface::InterfaceSelector position = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_POSITION);
	GraphicalInterface::InterfaceSelector game_team = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_TEAM);
	GraphicalInterface::InterfaceSelector mode = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_MODE);
	GraphicalInterface::InterfaceSelector game_round = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_ROUND);

	if (game_round == GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_SKILL)
	{
		Skill auton = Skill(&core, cata, false);
		auton.run();
		return;
	}

	// team
	bool position_offset = game_team == GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED;

	// fist position scoring mode
	if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
		mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE)
	{
		AutonFirstScoring auton = AutonFirstScoring(&core, cata, position_offset);
		auton.run();
		printf("[Main]: #1 score, offset %d\n", position_offset);
	}
	// second position scoring mode
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2 &&
			 mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE)
	{
		AutonSecondScoring auton = AutonSecondScoring(&core, cata, position_offset);
		auton.run();
		printf("[Main]: #2 score, offset %d\n", position_offset);
	}
	// fist position support mode (WP)
	else if (position == GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1 &&
			 mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT)
	{
		AutonFirstSupport auton = AutonFirstSupport(&core, cata, position_offset);
		auton.run();
		printf("[Main]: #1 support, offset %d\n", position_offset);
	}
	// idle mode
	else if (mode == GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE)
	{
		printf("[Main]: idle\n");
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
void opcontrol()
{
	printf("[Main]: Opcontrol started\n");
	// std::vector<Coordinates> ctlpoint = {
	// 	Coordinates(10.030533885227815, 17.933841152045563, 0),
	// 	Coordinates(17.81679342721255, 17.933841152045563, 0),
	// 	Coordinates(44.992365946296516, 28.010177029908156, 0),
	// 	Coordinates(30.183205640953005, 42.81933733525167, 0),
	// 	Coordinates(38.27480869438812, 53.81170374746541, 0),
	// 	Coordinates(49.72519037377744, 44.95674191540434, 0),
	// 	Coordinates(66.21373999209804, 50.6055968772364, 0),
	// 	Coordinates(50.3358773966782, 66.17811596120586, 0),
	// 	Coordinates(49.877862129502624, 82.36132206807609, 0),
	// 	Coordinates(66.51908350354843, 66.63613122838144, 0),
	// 	Coordinates(81.9389308317927, 81.75063504517534, 0),
	// 	Coordinates(90.79389266385377, 91.52162741158754, 0),
	// };

	// CatmullRom catmull = CatmullRom(ctlpoint);
	// std::vector<Coordinates> path = catmull.get_processed_coordinates();
	std::shared_ptr<Odom> odometry = std::make_shared<Odom>(&core, OdomMode::MOTOR_IMU);
	odometry->setState(17.81679342721255, 17.933841152045563, 0);
	Chassis chassis = Chassis(&core, odometry);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	// chassis.followPath(path);
	while (true)
	{
		chassis.cheezyDrive(core.controller->getAnalog(Configuration::Controls::FORWARD_AXIS), core.controller->getAnalog(Configuration::Controls::TURN_AXIS));
		pros::delay(10);
	}
}
