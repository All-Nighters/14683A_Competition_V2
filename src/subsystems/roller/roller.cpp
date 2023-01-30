#include "main.h"

Roller::Roller() {

}

Roller::Roller(struct Core* core) {
    this->core = core;
    this->roller_task = std::move(std::make_unique<pros::Task>(this->roller_loop_trampoline, this, "roller loop"));
    printf("finished roller\n");
}

Roller::~Roller() {
    this->roller_task->remove();
    this->roller_task.reset(nullptr);
    this->core->roller->moveVoltage(0);
    printf("roller destroyed\n");
}

void Roller::roller_loop_trampoline(void* iparam) {
    if(iparam){
        Roller* that = static_cast<Roller*>(iparam);
        that->roller_loop();
        pros::delay(10);
    }
}

void Roller::rollto(RollerDirection direction) {
    this->target_direction = direction;
}

void Roller::roller_loop() {
    while (true) {
        switch (this->target_direction) {
            case RollerDirection::BACKWARD:
                // rotate to red
                this->core->roller->moveVoltage(8000);
                pros::delay(300);
                this->target_direction = RollerDirection::NONE;
                this->core->roller->moveVoltage(0);
                break;
            case RollerDirection::FORWARD:
                // rotate to blue
                this->core->roller->moveVoltage(-8000);
                pros::delay(300);
                this->target_direction = RollerDirection::NONE;
                this->core->roller->moveVoltage(0);
                break;
        }
    }
}