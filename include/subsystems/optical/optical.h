#pragma once
#include <vector>
#include "main.h"

class Optical {
private:
    struct Core* core;
public:
    Optical(struct Core* core);
    int get_rgb_nearest(std::vector<pros::c::optical_rgb_s_t> colors);
};