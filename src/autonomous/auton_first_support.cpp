#include "main.h"

AutonFirstSupport::AutonFirstSupport(Chassis* chassis, Catapult* catapul, bool offset) {
    this->chassis = chassis;
    this->catapult = catapult;
    this->offset = offset;
}

AutonFirstSupport::~AutonFirstSupport() {
    this->chassis->moveVelocity(0);
}