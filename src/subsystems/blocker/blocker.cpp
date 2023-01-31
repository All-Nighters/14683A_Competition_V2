#include "main.h"

Blocker::Blocker(struct Core* core) {
    this->core = core;
    this->core->blocker_left->set_value(false);
    this->core->blocker_right->set_value(false);
    this->core->blocker_top->set_value(false);
}
Blocker::~Blocker() {
    this->core->blocker_left->set_value(false);
    this->core->blocker_right->set_value(false);
    this->core->blocker_top->set_value(false);
}
void Blocker::deploy() {
    this->core->blocker_left->set_value(true);
    this->core->blocker_right->set_value(true);
    this->core->blocker_top->set_value(true);
}
void Blocker::close() {
    this->core->blocker_left->set_value(false);
    this->core->blocker_right->set_value(false);
    this->core->blocker_top->set_value(false);
}
