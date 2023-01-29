#include "main.h"

AutonSecondScoring::AutonSecondScoring(Chassis* chassis, Catapult* catapult, Intake* intake, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->intake = intake;
    this->offset = offset;
}

AutonSecondScoring::~AutonSecondScoring() {
    this->chassis->moveVelocity(0);
}

void AutonSecondScoring::run() {
    
}