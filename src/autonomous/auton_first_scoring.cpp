#include "main.h"

AutonFirstScoring::AutonFirstScoring(Chassis* chassis, Catapult* catapult, Intake* intake, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->intake = intake;
    this->offset = offset;
}

AutonFirstScoring::~AutonFirstScoring() {
    this->chassis->moveVelocity(0);
}

void AutonFirstScoring::run() {
    std::vector<Coordinates> first_three_disk_path;
    if (this->offset) {
        // this->chassis->odom->setState(100-9.166666666666666, 100-27.5, 0);
        // this->chassis->simpleMoveToPoint(100-17.12962962962963, 100-26.85185185185185);
    } else {
        std::vector<Coordinates> first_three_disk_path = {
            Coordinates(12.314814814814815, 23.148148148148145, 0),
            Coordinates(14.351851851851851, 28.98148148148148, 0),
            Coordinates(25.925925925925924, 42.31481481481481, 0),
            Coordinates(33.33333333333333, 49.629629629629626, 0)
        };

        this->chassis->odom->setState(9.166666666666666, 27.5, 0);
        this->intake->turn_on();
        this->chassis->followPath(first_three_disk_path, true);
    }
    
}