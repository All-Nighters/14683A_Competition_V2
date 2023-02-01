#include "main.h"

AutonFirstScoring::AutonFirstScoring(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry, 350, 1, 500);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->offset = offset;
}

AutonFirstScoring::~AutonFirstScoring() {
    this->chassis_ptr->moveVelocity(0);
}

void AutonFirstScoring::run() {
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    this->catapult_ptr->set_boost(true);
    std::vector<Coordinates> first_two_disk_ctlpoint = {
        Coordinates(14.722222222222221,26.574074074074073,0),
        Coordinates(14.722222222222221,26.574074074074073,0),
        Coordinates(16.48148148148148,33.24074074074074,0),
        Coordinates(20.462962962962962,39.07407407407407,0),
        Coordinates(28.333333333333332, 44.35185185185185,0),
        Coordinates(28.333333333333332, 44.35185185185185,0),
    };
    std::vector<Coordinates> last_middle_ctlpoint = {
        Coordinates(32.5,60.09259259259259,0),
        Coordinates(32.5,60.09259259259259,0),
        Coordinates(37.68518518518518,57.59259259259259,0),
        Coordinates(50.46296296296296,47.59259259259259,0),

    };

    CatmullRom first_two_disk_catmull = CatmullRom(first_two_disk_ctlpoint);
    std::vector<Coordinates> first_two_disk_path = first_two_disk_catmull.get_processed_coordinates();
    CatmullRom last_middle_catmull = CatmullRom(last_middle_ctlpoint);
    std::vector<Coordinates> last_middle_path = last_middle_catmull.get_processed_coordinates();
    
    // start
    this->chassis_ptr->odom->setState(11.11111111111111, 27.5, -15.581617233912247);
    pros::delay(1500);
    // this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    this->roller_ptr->rollto(RollerDirection::FORWARD);
    this->chassis_ptr->moveVelocity(-50);
    pros::delay(300);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    this->catapult_ptr->fire(250);
    this->chassis_ptr->moveVelocity(400);
    pros::delay(400);
    this->chassis_ptr->moveVoltage(0);
    pros::delay(500);

    
    

    // shoot 2 disks
    this->chassis_ptr->moveVelocity(-50);
    pros::delay(400);
    this->chassis_ptr->moveVelocity(0);
    this->chassis_ptr->faceAngle(-135);
    this->chassis_ptr->followPath(first_two_disk_path, true);
    pros::delay(200);
    this->intake_ptr->turn_on();
    this->chassis_ptr->faceAngle(-135);
    this->chassis_ptr->moveDistance(-1);
    // pros::delay(500);
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    this->chassis_ptr->moveVoltage(4000);
    pros::delay(500);
    this->chassis_ptr->moveVoltage(0);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    this->intake_ptr->turn_off();
    this->catapult_ptr->fire();
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    pros::delay(500);


    // shoot 2 disks
    this->intake_ptr->turn_on();
    this->chassis_ptr->simpleMoveToPointBackwards(31.814814814814813, 61.01851851851851);
    this->chassis_ptr->moveVoltage(3000);
    pros::delay(600);

    this->chassis_ptr->followPath(last_middle_path, false);
    this->intake_ptr->turn_off();
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1], -3);
    this->catapult_ptr->fire(450);
    this->chassis_ptr->moveVoltage(3000);
    pros::delay(600);
    this->chassis_ptr->moveVoltage(0);
    this->catapult_ptr->wait_until_reloaded();
    
}