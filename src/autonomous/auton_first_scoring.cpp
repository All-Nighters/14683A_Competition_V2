#include "main.h"

AutonFirstScoring::AutonFirstScoring(struct Core* core, std::shared_ptr<Catapult> cata, bool offset) {
    Odom odometry = Odom(core, OdomMode::MIDDLETW_IMU);
    Chassis chassis = Chassis(core);
    Intake intake = Intake(core);
    Roller roller = Roller(core);

    this->odometry    = std::make_shared<Odom>(core, OdomMode::MIDDLETW_IMU);
    this->catapult_ptr = cata;
    this->chassis_ptr = std::make_unique<Chassis>(core, this->odometry);
    this->intake_ptr = std::make_unique<Intake>(core);
    this->roller_ptr = std::make_unique<Roller>(core);
    this->offset = offset;
}

AutonFirstScoring::~AutonFirstScoring() {
    this->chassis_ptr->moveVelocity(0);
}

void AutonFirstScoring::run() {
    this->catapult_ptr->set_boost(true);
    if (this->offset) { // red
        std::vector<Coordinates> first_three_disk_path = {
            Coordinates(100-12.314814814814815, 100-23.148148148148145, 0),
            Coordinates(100-14.351851851851851, 100-28.98148148148148, 0),
            Coordinates(100-25.925925925925924, 100-42.31481481481481, 0),
            Coordinates(100-33.33333333333333,  100-49.629629629629626, 0),
            Coordinates(100-36.48148148148148,  100-53.79629629629629, 0),
        };
        std::vector<Coordinates> last_three_disk_ctlpoint = {
            Coordinates(100-30.09259259259259, 100-44.166666666666664, 0),
            Coordinates(100-33.7037037037037,  100-50.09259259259259, 0),
            Coordinates(100-38.148148148148145,100-57.77777777777777, 0),
            Coordinates(100-33.425925925925924,100-62.59259259259259, 0),
            Coordinates(100-27.87037037037037, 100-62.31481481481481, 0),
            Coordinates(100-18.24074074074074, 100-62.22222222222222, 0),
            Coordinates(100-10.37037037037037, 100-62.31481481481481, 0),
            Coordinates(100-10.37037037037037, 100-62.31481481481481, 0),
        };
        std::vector<Coordinates> last_middle_ctlpoint = {
            Coordinates(100-11.944444444444445, 100-58.33333333333333, 0),
            Coordinates(100-18.88888888888889,  100-58.79629629629629, 0),
            Coordinates(100-33.61111111111111,  100-58.51851851851851, 0),
            Coordinates(100-45.37037037037037,  100-55.55555555555555, 0),
            Coordinates(100-58.05555555555555,  100-42.68518518518518, 0),
        };
        CatmullRom last_three_disk_catmull = CatmullRom(last_three_disk_ctlpoint);
        CatmullRom last_middle_catmull = CatmullRom(last_middle_ctlpoint);
        std::vector<Coordinates> last_three_disk_path = last_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> last_middle_path = last_middle_catmull.get_processed_coordinates();

        // start
        this->chassis_ptr->odom->setState(100-9.166666666666666, 100-27.5, 180);
        
        this->roller_ptr->rollto(RollerDirection::FORWARD);
        this->chassis_ptr->moveVelocity(-100);
        pros::delay(500);
        this->chassis_ptr->moveVelocity(0);

        // first three disk
        this->intake_ptr->turn_on();
        this->chassis_ptr->followPath(first_three_disk_path, true);
        this->chassis_ptr->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
        pros::delay(500);
        this->intake_ptr->turn_off();
        this->catapult_ptr->fire();
        this->catapult_ptr->wait_until_reloaded();

        // last three disk
        this->intake_ptr->turn_on();
        this->chassis_ptr->followPath(last_three_disk_path, true);
        pros::delay(600);

        this->chassis_ptr->followPath(last_middle_path);
        this->intake_ptr->turn_off();
        this->chassis_ptr->faceCoordinate(Constants::Field::RED_HIGH_GOAL_PCT[0], Constants::Field::RED_HIGH_GOAL_PCT[1]);
        pros::delay(500);
        this->catapult_ptr->fire();
        this->catapult_ptr->wait_until_reloaded();

    } else { // blue
        std::vector<Coordinates> first_three_disk_path = {
            Coordinates(12.314814814814815, 23.148148148148145, 0),
            Coordinates(14.351851851851851, 28.98148148148148, 0),
            Coordinates(25.925925925925924, 42.31481481481481, 0),
            Coordinates(33.33333333333333, 49.629629629629626, 0),
            Coordinates(36.48148148148148, 53.79629629629629, 0)
        };
        std::vector<Coordinates> last_three_disk_ctlpoint = {
            Coordinates(30.09259259259259,44.166666666666664, 0),
            Coordinates(33.7037037037037,50.09259259259259, 0),
            Coordinates(38.148148148148145,57.77777777777777, 0),
            Coordinates(33.425925925925924,62.59259259259259, 0),
            Coordinates(27.87037037037037,62.31481481481481, 0),
            Coordinates(18.24074074074074,62.22222222222222, 0),
            Coordinates(10.37037037037037,62.31481481481481, 0),
            Coordinates(10.37037037037037,62.31481481481481, 0),
        };
        std::vector<Coordinates> last_middle_ctlpoint = {
            Coordinates(11.944444444444445, 58.33333333333333, 0),
            Coordinates(18.88888888888889, 58.79629629629629, 0),
            Coordinates(33.61111111111111, 58.51851851851851, 0),
            Coordinates(45.37037037037037, 55.55555555555555, 0),
            Coordinates(58.05555555555555, 42.68518518518518, 0),
        };
        CatmullRom last_three_disk_catmull = CatmullRom(last_three_disk_ctlpoint);
        CatmullRom last_middle_catmull = CatmullRom(last_middle_ctlpoint);
        std::vector<Coordinates> last_three_disk_path = last_three_disk_catmull.get_processed_coordinates();
        std::vector<Coordinates> last_middle_path = last_middle_catmull.get_processed_coordinates();

        // start
        this->chassis_ptr->odom->setState(9.166666666666666, 27.5, 0);
        
        this->roller_ptr->rollto(RollerDirection::FORWARD);
        this->chassis_ptr->moveVelocity(-100);
        pros::delay(500);
        this->chassis_ptr->moveVelocity(0);

        // shoot 3 disks
        this->intake_ptr->turn_on();
        this->chassis_ptr->followPath(first_three_disk_path, true);
        this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
        pros::delay(500);
        this->intake_ptr->turn_off();
        this->catapult_ptr->fire();
        this->catapult_ptr->wait_until_reloaded();

        // shoot 3 disks
        this->intake_ptr->turn_on();
        this->chassis_ptr->followPath(last_three_disk_path, true);
        pros::delay(600);

        this->chassis_ptr->followPath(last_middle_path);
        this->intake_ptr->turn_off();
        this->chassis_ptr->faceCoordinate(Constants::Field::BLUE_HIGH_GOAL_PCT[0], Constants::Field::BLUE_HIGH_GOAL_PCT[1]);
        pros::delay(500);
        this->catapult_ptr->fire();
        this->catapult_ptr->wait_until_reloaded();
    }
    
}