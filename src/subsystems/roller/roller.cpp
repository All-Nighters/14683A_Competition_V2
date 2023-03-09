#include "main.h"

Roller::Roller() {

}

Roller::Roller(struct Core* core) {
    this->core = core;
    printf("[Roller]: Roller created\n");
}

Roller::~Roller() {
    this->core->roller->moveVoltage(0);
    printf("Roller destroyed\n");
}

void Roller::rollto(RollerDirection direction) {
    switch (direction) {
        case RollerDirection::BACKWARD:
            this->core->roller->moveVoltage(10000);
            break;
        case RollerDirection::FORWARD:
            this->core->roller->moveVoltage(-10000);
            break;
    }
}

void Roller::stop() {
    this->core->roller->moveVoltage(0);
}