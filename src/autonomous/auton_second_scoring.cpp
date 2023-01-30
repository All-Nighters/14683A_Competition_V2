#include "main.h"

AutonSecondScoring::AutonSecondScoring(Chassis* chassis, Catapult* catapult, Intake* intake, Roller* roller, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->intake = intake;
    this->offset = offset;
    this->roller = roller;
}

AutonSecondScoring::~AutonSecondScoring() {
    this->chassis->moveVelocity(0);
}

void AutonSecondScoring::run() {
    if (this->offset) { // red
        std::vector<Coordinates> first_three_disk_ctlpoint = {
            Coordinates(100-70.27777777777777,  100-86.38888888888889,0),
            Coordinates(100-65.92592592592592,  100-82.59259259259258,0),
            Coordinates(100-58.79629629629629,  100-74.16666666666666,0),
            Coordinates(100-50.55555555555555,  100-65.37037037037037,0),
            Coordinates(100-45.09259259259259,  100-57.87037037037037,0),
            Coordinates(100-40.46296296296296,  100-58.7037037037037,0),
            Coordinates(100-28.888888888888886, 100-70.27777777777777,0),
        };
        std::vector<Coordinates> back_to_roller_ctlpoint = {
            Coordinates(100-40.46296296296296,  100-58.7037037037037,0),
            Coordinates(100-40.46296296296296,  100-58.7037037037037,0),
            Coordinates(100-43.79629629629629,  100-66.75925925925925,0),
            Coordinates(100-49.166666666666664, 100-74.35185185185185,0),
            Coordinates(100-58.148148148148145, 100-75.83333333333333,0),
            Coordinates(100-66.85185185185185,  100-79.9074074074074,0),
            Coordinates(100-72.87037037037037,  100-86.01851851851852,0),
            Coordinates(100-73.14814814814814,  100-90.64814814814814,0),
            Coordinates(100-73.24074074074073,  100-98.79629629629629,0),
        };

        CatmullRom first_three_disk_catmull = CatmullRom(first_three_disk_ctlpoint);
        CatmullRom back_to_roller_catmull = CatmullRom(back_to_roller_ctlpoint);
        std::vector<Coordinates> first_three_disk_path = first_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> back_to_roller_path = back_to_roller_catmull.get_processed_coordinates();

        // start
        this->chassis->odom->setState(100-60.925925925925924, 100-90.55555555555556, 70.37755003019457);
        // shoot preload
        this->chassis->moveDistance(30);

        // shoot 3 disks
        this->intake->turn_on();
        this->chassis->followPath(first_three_disk_path, true);
        this->intake->turn_off();
        this->chassis->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
        
        // turn roller
        this->chassis->followPath(back_to_roller_path, true);
        this->chassis->faceAngle(0);
        this->roller->rollto(RollerDirection::FORWARD);
        this->chassis->moveVelocity(-100);
        pros::delay(500);
        this->chassis->moveVelocity(0);


    } else { // blue
        std::vector<Coordinates> first_three_disk_ctlpoint = {
            Coordinates(70.27777777777777,86.38888888888889,0),
            Coordinates(65.92592592592592,82.59259259259258,0),
            Coordinates(58.79629629629629,74.16666666666666,0),
            Coordinates(50.55555555555555,65.37037037037037,0),
            Coordinates(45.09259259259259,57.87037037037037,0),
            Coordinates(40.46296296296296,58.7037037037037,0),
            Coordinates(28.888888888888886,70.27777777777777,0),
        };
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
        
        CatmullRom first_three_disk_catmull = CatmullRom(first_three_disk_ctlpoint);
        CatmullRom back_to_roller_catmull = CatmullRom(back_to_roller_ctlpoint);
        std::vector<Coordinates> first_three_disk_path = first_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> back_to_roller_path = back_to_roller_catmull.get_processed_coordinates();

        // start
        this->chassis->odom->setState(60.925925925925924, 90.55555555555556, -70.37755003019457);
        // shoot preload
        this->chassis->moveDistance(30);

        // shoot 3 disks
        this->intake->turn_on();
        this->chassis->followPath(first_three_disk_path, true);
        this->intake->turn_off();
        this->chassis->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
        
        // turn roller
        this->chassis->followPath(back_to_roller_path, true);
        this->chassis->faceAngle(0);
        this->roller->rollto(RollerDirection::FORWARD);
        this->chassis->moveVelocity(-100);
        pros::delay(500);
        this->chassis->moveVelocity(0);


    }
}