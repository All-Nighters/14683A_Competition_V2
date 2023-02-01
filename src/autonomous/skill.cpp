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
    std::vector<Coordinates> second_three_disks_ctlpoint = {
        Coordinates(62.22222222222222	,10.462962962962962	,0),
        Coordinates(62.22222222222222	,10.462962962962962	,0),
        Coordinates(51.388888888888886	,11.759259259259258	,0),
        Coordinates(39.629629629629626	,14.351851851851851	,0),
        Coordinates(33.05555555555556	,18.425925925925924	,0),
        Coordinates(26.85185185185185	,25.09259259259259	,0),
        Coordinates(17.037037037037035	,37.68518518518518	,0),
    };
    CatmullRom second_three_disks_catmull = CatmullRom(second_three_disks_ctlpoint);
    std::vector<Coordinates> second_three_disks_path = second_three_disks_catmull.get_processed_coordinates();
    
    std::vector<Coordinates> third_three_disks_ctlpoint = {
        Coordinates(11.481481481481481	,63.51851851851851	,0),
        Coordinates(11.574074074074073	,57.5	,0),
        Coordinates(17.5	,50.55555555555555	,0),
        Coordinates(25.833333333333332	,43.05555555555555	,0),
        Coordinates(40.18518518518518	,29.907407407407405	,0),

    };
    CatmullRom third_three_disks_catmull = CatmullRom(third_three_disks_ctlpoint);
    std::vector<Coordinates> third_three_disks_path = third_three_disks_catmull.get_processed_coordinates();

    std::vector<Coordinates> fourth_three_disks_ctlpoint = {
        Coordinates(7.592592592592593	,65.83333333333333	,0),
        Coordinates(10.925925925925926	,57.68518518518518	,0),
        Coordinates(17.87037037037037	,49.907407407407405	,0),
        Coordinates(26.48148148148148	,42.5	,0),
        Coordinates(34.166666666666664	,34.35185185185185	,0),
        Coordinates(42.03703703703704	,26.388888888888886	,0),
        Coordinates(45.18518518518518	,14.629629629629628	,0),
        Coordinates(58.33333333333333	,12.407407407407407	,0),
        Coordinates(77.5925925925926	,13.796296296296296	,0),

    };
    CatmullRom fourth_three_disks_catmull = CatmullRom(fourth_three_disks_ctlpoint);
    std::vector<Coordinates> fourth_three_disks_path = fourth_three_disks_catmull.get_processed_coordinates();

    std::vector<Coordinates> fifth_three_disks_ctlpoint = {
        Coordinates(10.648148148148147	,65.92592592592592	,0),
        Coordinates(10.925925925925926	,57.87037037037037	,0),
        Coordinates(13.425925925925926	,46.29629629629629	,0),
        Coordinates(24.72222222222222	,46.01851851851852	,0),
        Coordinates(34.629629629629626	,50.55555555555555	,0),
        Coordinates(42.03703703703704	,58.24074074074074	,0),
        Coordinates(49.907407407407405	,66.29629629629629	,0),
        Coordinates(58.05555555555555	,74.9074074074074	,0),
        Coordinates(57.96296296296296	,82.77777777777777	,0),
        Coordinates(41.57407407407407	,87.87037037037037	,0),
        Coordinates(31.85185185185185	,87.12962962962962	,0),

    };
    CatmullRom fifth_three_disks_catmull = CatmullRom(fifth_three_disks_ctlpoint);
    std::vector<Coordinates> fifth_three_disks_path = fifth_three_disks_catmull.get_processed_coordinates();

    // start
    this->chassis_ptr->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    this->chassis_ptr->odom->setState(10.74074074074074, 25.833333333333332, 0);
    pros::delay(200);
    
    /*
    First three disks
    */
    this->roller_ptr->rollto(RollerDirection::BACKWARD);
    this->chassis_ptr->moveVelocity(-50);
    pros::delay(300);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    this->intake_ptr->turn_on();
    this->chassis_ptr->simpleMoveToPointBackwards(21.203703703703702, 14.074074074074073);
    this->chassis_ptr->faceAngle(90);

    this->chassis_ptr->moveVelocity(-200);
    pros::delay(300);
    this->chassis_ptr->moveVoltage(0);
    this->intake_ptr->turn_off();

    this->chassis_ptr->simpleMoveToPoint(58.425925925925924, 12.592592592592592);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    Second three disks
    */
    this->chassis_ptr->followPath(second_three_disks_path, true);
    this->chassis_ptr->moveDistance(-10);
    this->chassis_ptr->simpleMoveToPoint(12.87037037037037, 59.25925925925925);
    this->chassis_ptr->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    Third three disks
    */
    this->chassis_ptr->followPath(third_three_disks_path, true);
    pros::delay(500);
    this->intake_ptr->turn_on();
    this->chassis_ptr->moveDistance(-10);
    this->intake_ptr->turn_off();
    this->chassis_ptr->simpleMoveToPoint(12.87037037037037, 59.25925925925925);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    Fourth three disks
    */
    this->intake_ptr->turn_on();
    this->chassis_ptr->followPath(fourth_three_disks_path, true);
    pros::delay(1000);
    this->intake_ptr->turn_off();
    this->chassis_ptr->moveVelocity(-200);
    pros::delay(500);
    this->chassis_ptr->simpleMoveToPoint(12.87037037037037, 59.25925925925925);
    this->chassis_ptr->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    Fifth three disks
    */
    this->intake_ptr->turn_on();
    this->chassis_ptr->followPath(fifth_three_disks_path, true);
    this->intake_ptr->turn_off();
    this->chassis_ptr->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    final two rollers
    */ 
    this->chassis_ptr->simpleMoveToPoint(77.96296296296296, 78.7037037037037);
    this->chassis_ptr->faceAngle(-90);
    pros::delay(500);
    this->roller_ptr->rollto(RollerDirection::BACKWARD);
    this->chassis_ptr->moveVelocity(-100);
    pros::delay(1000);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    this->chassis_ptr->simpleMoveToPoint(77.96296296296296, 78.7037037037037);
    this->chassis_ptr->faceAngle(180);
    pros::delay(500);
    this->roller_ptr->rollto(RollerDirection::BACKWARD);
    this->chassis_ptr->moveVelocity(-100);
    pros::delay(1000);
    this->chassis_ptr->moveVelocity(0);
    this->roller_ptr->stop();

    /*
    expansion
    */ 
    this->chassis_ptr->simpleMoveToPoint(77.96296296296296, 78.7037037037037);
    this->chassis_ptr->turnAngle(45);
    this->expansion_ptr->deploy();
}