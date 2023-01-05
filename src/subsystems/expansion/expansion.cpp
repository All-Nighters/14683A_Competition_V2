#include "main.h"

Expansion::Expansion(struct Core* core) {
    this->core = core;
    this->core->expansion->set_value(false);
}
Expansion::~Expansion() {
    this->core->expansion->set_value(false);
}
void Expansion::deploy() {
    this->core->expansion->set_value(true);
}
