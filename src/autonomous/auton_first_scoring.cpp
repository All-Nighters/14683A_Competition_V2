#include "main.h"

AutonFirstScoring::AutonFirstScoring(Chassis* chassis, Catapult* catapul, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->offset = offset;
}

AutonFirstScoring::~AutonFirstScoring() {
    this->chassis->moveVelocity(0);
}

void AutonFirstScoring::run() {
    
}