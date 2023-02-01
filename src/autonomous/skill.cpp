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
        Coordinates(57.96296296296296	,10.37037037037037	,0),
        Coordinates(57.96296296296296	,10.37037037037037	,0),
        Coordinates(59.629629629629626	,18.703703703703702	,0),
        Coordinates(59.35185185185185	,25.833333333333332	,0),
        Coordinates(59.166666666666664	,34.53703703703704	,0),
        Coordinates(58.98148148148148	,43.888888888888886	,0),
    };
    CatmullRom second_three_disks_catmull = CatmullRom(second_three_disks_ctlpoint);
    std::vector<Coordinates> second_three_disks_path = second_three_disks_catmull.get_processed_coordinates();
    
    std::vector<Coordinates> third_three_disks_ctlpoint = {
        Coordinates(58.33333333333333	,42.407407407407405	,0),
        Coordinates(58.33333333333333	,42.407407407407405	,0),
        Coordinates(64.72222222222221	,40.46296296296296	,0),
        Coordinates(73.51851851851852	,39.81481481481481	,0),
        Coordinates(82.31481481481481	,40.27777777777778	,0),
        Coordinates(85.09259259259258	,46.388888888888886	,0),
        Coordinates(85.27777777777777	,54.166666666666664	,0),
        Coordinates(85.18518518518518	,61.01851851851851	,0),

    };
    CatmullRom third_three_disks_catmull = CatmullRom(third_three_disks_ctlpoint);
    std::vector<Coordinates> third_three_disks_path = third_three_disks_catmull.get_processed_coordinates();

    std::vector<Coordinates> fourth_three_disks_ctlpoint = {
        Coordinates(88.42592592592592	,41.11111111111111	,0),
        Coordinates(88.42592592592592	,41.11111111111111	,0),
        Coordinates(85.0	,48.24074074074074	,0),
        Coordinates(80.0	,53.05555555555555	,0),
        Coordinates(74.44444444444444	,57.5	,0),
        Coordinates(63.51851851851851	,63.79629629629629	,0),


    };
    CatmullRom fourth_three_disks_catmull = CatmullRom(fourth_three_disks_ctlpoint);
    std::vector<Coordinates> fourth_three_disks_path = fourth_three_disks_catmull.get_processed_coordinates();

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
    this->chassis_ptr->simpleMoveToPointBackwards(59.35185185185185, 10.648148148148147);
    this->chassis_ptr->faceAngle(-95);
    this->chassis_ptr->followPath(second_three_disks_path, true)
    this->chassis_ptr->simpleMoveToPointBackwards(59.35185185185185, 10.648148148148147);
    this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
    pros::delay(500);
    // fire

    /*
    Third three disks
    */
    this->chassis_ptr->simpleMoveToPoint(57.22222222222222, 40.46296296296296);
    this->chassis_ptr->faceAngle(-175);
    this->chassis_ptr->followPath(second_three_disks_path, true)
    this->chassis_ptr->simpleMoveToPoint(88.33333333333333, 42.03703703703704);
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