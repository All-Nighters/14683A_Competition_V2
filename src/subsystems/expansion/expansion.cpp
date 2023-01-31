#include "main.h"

Expansion::Expansion(struct Core* core) {
    this->core = core;
    this->core->expansion_left->set_value(false);
    this->core->expansion_right->set_value(false);
}
Expansion::~Expansion() {
    this->core->expansion_left->set_value(false);
    this->core->expansion_right->set_value(false);
}
void Expansion::deploy() {
    this->core->expansion_left->set_value(true);
    this->core->expansion_right->set_value(true);
}
