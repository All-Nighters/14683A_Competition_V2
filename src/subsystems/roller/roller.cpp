#include "main.h"

Roller::Roller(struct Core* core) {
    this->core = core;
    this->roller_task = std::move(std::make_unique<pros::Task>(this->roller_loop_trampoline, this, "roller loop"));
}

Roller::~Roller() {
    this->roller_task->remove();
    this->roller_task.reset(nullptr);
    this->core->roller->moveVoltage(0);
}

void Roller::roller_loop_trampoline(void* iparam) {
    if(iparam){
        Roller* that = static_cast<Roller*>(iparam);
        that->roller_loop();
        pros::delay(10);
    }
}

void Roller::rollto(RollerColor color) {
    this->target_color = color;
}

void Roller::roller_loop() {
    while (true) {
        switch (this->target_color) {
            case RollerColor::RED:
                // rotate to red
                this->core->roller->moveVoltage(12000);
                pros::delay(500);
                this->target_color = RollerColor::NONE;
                this->core->roller->moveVoltage(0);
                break;
            case RollerColor::BLUE:
                // rotate to blue
                this->core->roller->moveVoltage(-12000);
                pros::delay(500);
                this->target_color = RollerColor::NONE;
                this->core->roller->moveVoltage(0);
                break;
        }
    }
}