#include "main.h"

Intake::Intake(struct Core* core) {
    this->core = core;
}

Intake::~Intake() {
    this->turn_off();
}
void Intake::turn_on() {
    this->core->intake->moveVoltage(-12000);
}
void Intake::turn_off() {
    this->core->intake->moveVoltage(0);
}

