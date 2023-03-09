#include "main.h"

Intake::Intake(struct Core* core) {
    this->core = core;
    printf("[Intake]: Intake created\n");
}

Intake::~Intake() {
    this->turn_off();
    printf("[Intake]: Intake destroyed\n");
}
void Intake::turn_on() {
    if (this->core->catapult_load_sensor->get_value() == 1) {
        this->core->intake->moveVoltage(-10000);
    } else {
        this->core->intake->moveVoltage(-10000);
    }
}
void Intake::turn_on_rev() {
    this->core->intake->moveVoltage(10000);
}
void Intake::turn_off() {
    this->core->intake->moveVoltage(0);
}

