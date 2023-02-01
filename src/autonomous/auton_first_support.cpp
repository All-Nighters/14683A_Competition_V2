#include "main.h"

AutonFirstSupport::AutonFirstSupport(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->offset = offset;
}
AutonFirstSupport::~AutonFirstSupport() {
    this->chassis_ptr->moveVelocity(0);
}

void AutonFirstSupport::run() {
    this->catapult_ptr->set_boost(true);
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    std::vector<Coordinates> first_three_disk_ctlpoint = {
        Coordinates(10.648148148148147,29.814814814814813,0),
        Coordinates(10.648148148148147,29.814814814814813,0),
        Coordinates(17.59259259259259,40.37037037037037,0),
        Coordinates(27.87037037037037,47.31481481481481,0),
        Coordinates(37.68518518518518,45.925925925925924,0),
        Coordinates(52.87037037037037,39.53703703703704,0),
    };
    std::vector<Coordinates> last_three_disk_ctlpoint = {
        Coordinates(43.61111111111111,54.907407407407405,0),
        Coordinates(43.61111111111111,54.907407407407405,0),
        Coordinates(51.85185185185185,61.75925925925925,0),
        Coordinates(60.09259259259259,69.35185185185185,0),
        Coordinates(65.92592592592592,75.92592592592592,0),
        Coordinates(75.0925925925926,85.37037037037037,0),
        Coordinates(75.83333333333333,97.59259259259258,0),


    };
    CatmullRom first_three_disk_catmull = CatmullRom(first_three_disk_ctlpoint);
    CatmullRom last_three_disk_catmull = CatmullRom(last_three_disk_ctlpoint);
    std::vector<Coordinates> first_three_disk_path = first_three_disk_catmull.get_processed_coordinates();
    std::vector<Coordinates> last_three_disk_path = last_three_disk_catmull.get_processed_coordinates();

    // start
    this->chassis_ptr->odom->setState(9.166666666666666, 27.5, 0);

    this->roller_ptr->rollto(RollerDirection::FORWARD);
    this->chassis_ptr->moveVelocity(-100);
    pros::delay(300);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    // shoot 
    this->chassis_ptr->moveDistance(10);
    this->chassis_ptr->followPath(first_three_disk_path);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1], -3);
    pros::delay(500);
    this->catapult_ptr->fire();
    pros::delay(200);
    this->catapult_ptr->set_boost(false);
    this->catapult_ptr->wait_until_reloaded();


    this->intake_ptr->turn_on();
    this->chassis_ptr->followPath(last_three_disk_path, true);
    this->chassis_ptr->faceAngle(-90);
    this->chassis_ptr->moveVelocity(-100, -200);
    pros::delay(1500);
    this->chassis_ptr->moveVelocity(0);
    this->intake_ptr->turn_off();
}