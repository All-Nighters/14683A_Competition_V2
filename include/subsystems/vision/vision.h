#pragma once
#include "main.h"

class Vision {
private:
    struct Core* core;
public:
    Vision(struct Core* core);
    float get_direction(pros::vision_signature_s_t signature_pointer);
};