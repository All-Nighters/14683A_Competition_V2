#pragma once
#include <vector>
#include "main.h"

class Vision {
private:
    struct Core* core;
public:
    Vision(struct Core* core);
    void  set_signatures(std::vector<pros::vision_signature_s_t> signatures);
    float get_direction();
};