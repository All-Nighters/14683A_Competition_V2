#include "main.h"

AutonFirstSupport::AutonFirstSupport(Chassis* chassis, Catapult* catapult, Intake* intake, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->intake = intake;
    this->offset = offset;
}

AutonFirstSupport::~AutonFirstSupport() {
    this->chassis->moveVelocity(0);
}

void AutonFirstSupport::run() {
    
}