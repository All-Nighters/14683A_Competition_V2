#include "main.h"

AutonSecondScoring::AutonSecondScoring(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->offset = offset;
}

AutonSecondScoring::~AutonSecondScoring() {
    this->chassis_ptr->moveVelocity(0);
}

void AutonSecondScoring::run() {
    this->catapult_ptr->set_boost(true);
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    std::vector<Coordinates> back_to_roller_ctlpoint = {
        Coordinates(40.46296296296296,58.7037037037037,0),
        Coordinates(40.46296296296296,58.7037037037037,0),
        Coordinates(43.79629629629629,66.75925925925925,0),
        Coordinates(49.166666666666664,74.35185185185185,0),
        Coordinates(58.148148148148145,75.83333333333333,0),
        Coordinates(66.85185185185185,79.9074074074074,0),
        Coordinates(72.87037037037037,86.01851851851852,0),
        Coordinates(73.14814814814814,90.64814814814814,0),
        Coordinates(73.24074074074073,98.79629629629629,0),
    };
    
    CatmullRom back_to_roller_catmull = CatmullRom(back_to_roller_ctlpoint);
    std::vector<Coordinates> back_to_roller_path = back_to_roller_catmull.get_processed_coordinates();

    // start
    this->chassis_ptr->odom->setState(60.925925925925924, 90.55555555555556, -70.37755003019457);
    pros::delay(1000);
    // shoot preload
    this->catapult_ptr->fire(200);
    this->chassis_ptr->moveVelocity(399);
    pros::delay(500);
    this->chassis_ptr->moveVoltage(0);
    pros::delay(700);

    // shoot 2 disks
    this->intake_ptr->turn_on();
    this->chassis_ptr->simpleMoveToPointBackwards(46.85185185185185, 62.96296296296296, 3000);
    pros::delay(500);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    this->intake_ptr->turn_off();
    this->chassis_ptr->moveVelocity(390);
    this->catapult_ptr->fire(100);
    pros::delay(200);
    this->chassis_ptr->moveVoltage(0);
    pros::delay(300);

    // turn roller
    this->chassis_ptr->followPath(back_to_roller_path, true);
    this->roller_ptr->rollto(RollerDirection::FORWARD);
    this->chassis_ptr->moveVelocity(-50, -150);
    pros::delay(1000);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();
}