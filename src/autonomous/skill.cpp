#include "main.h"

Skill::Skill(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry, 350, 1, 500);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->offset = offset;
}

Skill::~Skill() {
    this->chassis_ptr->moveVelocity(0);
}

void Skill::run() {
    
}