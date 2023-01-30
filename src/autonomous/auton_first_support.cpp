#include "main.h"

AutonFirstSupport::AutonFirstSupport(Chassis* chassis, Catapult* catapult, Intake* intake, Roller* roller, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->intake = intake;
    this->offset = offset;
    this->roller = roller;
}

AutonFirstSupport::~AutonFirstSupport() {
    this->chassis->moveVelocity(0);
}

void AutonFirstSupport::run() {
    if (this->offset) { // red
        std::vector<Coordinates> first_three_disk_ctlpoint = {
            Coordinates(100-10.092592592592592 ,100-28.055555555555554,0),
            Coordinates(100-9.907407407407407  ,100-28.148148148148145,0),
            Coordinates(100-20.462962962962962 ,100-32.96296296296296,0),
            Coordinates(100-25.925925925925924 ,100-42.5,0),
            Coordinates(100-33.7037037037037   ,100-50.27777777777777,0),
            Coordinates(100-53.61111111111111  ,100-37.03703703703704,0),
        };
        std::vector<Coordinates> last_three_disk_ctlpoint = {
            Coordinates(100-33.7037037037037   ,100-50.27777777777777,0),
            Coordinates(100-33.7037037037037   ,100-50.27777777777777,0),
            Coordinates(100-41.85185185185185  ,100-58.61111111111111,0),
            Coordinates(100-50.0               ,100-66.29629629629629,0),
            Coordinates(100-57.87037037037037  ,100-74.53703703703704,0),
            Coordinates(100-68.88888888888889  ,100-81.57407407407408,0),
            Coordinates(100-71.94444444444444  ,100-88.14814814814814,0),
            Coordinates(100-72.12962962962962  ,100-93.98148148148148,0),
        };
        CatmullRom first_three_disk_catmull = CatmullRom(first_three_disk_ctlpoint);
        CatmullRom last_three_disk_catmull = CatmullRom(last_three_disk_ctlpoint);
        std::vector<Coordinates> first_three_disk_path = first_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> last_three_disk_path = last_three_disk_catmull.get_processed_coordinates();

        // start
        this->chassis->odom->setState(100-9.166666666666666, 100-27.5, 180);

        this->chassis->followPath(first_three_disk_path);
        this->chassis->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
        pros::delay(1000);
        this->chassis->followPath(last_three_disk_path);
        this->chassis->faceAngle(0);
        this->roller->rollto(RollerDirection::FORWARD);
        this->chassis->moveVelocity(-100);
        pros::delay(500);
        this->chassis->moveVelocity(0);

    } else { // blue
        std::vector<Coordinates> first_three_disk_ctlpoint = {
            Coordinates(10.092592592592592,28.055555555555554,0),
            Coordinates(9.907407407407407,28.148148148148145,0),
            Coordinates(20.462962962962962,32.96296296296296,0),
            Coordinates(25.925925925925924,42.5,0),
            Coordinates(33.7037037037037,50.27777777777777,0),
            Coordinates(53.61111111111111,37.03703703703704,0),
        };
        std::vector<Coordinates> last_three_disk_ctlpoint = {
            Coordinates(33.7037037037037,50.27777777777777,0),
            Coordinates(33.7037037037037,50.27777777777777,0),
            Coordinates(41.85185185185185,58.61111111111111,0),
            Coordinates(50.0,66.29629629629629,0),
            Coordinates(57.87037037037037,74.53703703703704,0),
            Coordinates(68.88888888888889,81.57407407407408,0),
            Coordinates(71.94444444444444,88.14814814814814,0),
            Coordinates(72.12962962962962,93.98148148148148,0),
        };
        CatmullRom first_three_disk_catmull = CatmullRom(first_three_disk_ctlpoint);
        CatmullRom last_three_disk_catmull = CatmullRom(last_three_disk_ctlpoint);
        std::vector<Coordinates> first_three_disk_path = first_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> last_three_disk_path = last_three_disk_catmull.get_processed_coordinates();

        // start
        this->chassis->odom->setState(9.166666666666666, 27.5, 0);

        // shoot 3 disks
        this->chassis->followPath(first_three_disk_path);
        this->chassis->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
        pros::delay(1000);

        // take 3 disks & get roller
        this->chassis->followPath(last_three_disk_path);
        this->chassis->faceAngle(0);
        this->roller->rollto(RollerDirection::FORWARD);
        this->chassis->moveVelocity(-100);
        pros::delay(500);
        this->chassis->moveVelocity(0);

    }
}