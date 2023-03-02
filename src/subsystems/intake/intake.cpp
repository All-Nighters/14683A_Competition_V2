#include "main.h"

Intake::Intake() {

}

Intake::Intake(struct Core* core) {
    this->core = core;
    printf("finished intake\n");
}

Intake::~Intake() {
    this->turn_off();
    printf("intake destroyed\n");
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

