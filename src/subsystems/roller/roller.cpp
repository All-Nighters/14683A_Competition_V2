#include "main.h"

Roller::Roller() {

}

Roller::Roller(struct Core* core) {
    this->core = core;
    printf("finished roller\n");
}

Roller::~Roller() {
    this->core->roller->moveVoltage(0);
    printf("roller destroyed\n");
}

void Roller::rollto(RollerDirection direction) {
    switch (direction) {
        case RollerDirection::BACKWARD:
            this->core->roller->moveVoltage(8000);
            break;
        case RollerDirection::FORWARD:
            this->core->roller->moveVoltage(-8000);
            break;
    }
}

void Roller::stop() {
    this->core->roller->moveVoltage(0);
}