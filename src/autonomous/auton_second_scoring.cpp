#include "main.h"

AutonSecondScoring::AutonSecondScoring(Chassis* chassis, Catapult* catapul, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->offset = offset;
}

AutonSecondScoring::~AutonSecondScoring() {
    this->chassis->moveVelocity(0);
}