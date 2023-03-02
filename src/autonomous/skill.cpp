#include "main.h"

Skill::Skill(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry, 350, 1, 500);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->expansion_ptr = std::make_unique<Expansion>(core);
    this->offset = offset;
}

Skill::~Skill() {
    this->chassis_ptr->moveVelocity(0);
}

void Skill::run() {
    this->catapult_ptr->set_boost(true);
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    this->chassis_ptr->odom->setState(11.11111111111111, 27.5, -15.581617233912247);
    pros::delay(1500);

    this->roller_ptr->rollto(RollerDirection::FORWARD);
    this->chassis_ptr->moveVelocity(-80);
    pros::delay(500);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    this->catapult_ptr->fire(250);
    this->chassis_ptr->moveVelocity(400);
    pros::delay(400);
    this->chassis_ptr->moveVoltage(0);
    pros::delay(500);
}