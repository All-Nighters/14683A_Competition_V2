#include "main.h"

Blocker::Blocker(struct Core* core) {
    this->core = core;
    this->core->blocker->set_value(false);
}
Blocker::~Blocker() {
    this->core->blocker->set_value(false);
}
void Blocker::deploy() {
    this->core->blocker->set_value(true);
}
