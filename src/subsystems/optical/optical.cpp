#include <cmath>
#include "main.h"

Optical::Optical(struct Core* core) {
    this->core = core;
}

int Optical::get_rgb_nearest(std::vector<pros::c::optical_rgb_s_t> colors) {
    pros::c::optical_rgb_s_t optical_rgb = this->core->optical_roller->get_rgb();
    int    nearest_index    = -1;
    double nearest_distance = -1.0f;
    for (int color_index = 0; color_index < colors.size(); color_index++) {
        pros::c::optical_rgb_s_t loop_rgb = colors[color_index];
        double distance_red   = optical_rgb.red   - loop_rgb.red;
        double distance_green = optical_rgb.green - loop_rgb.green;
        double distance_blue  = optical_rgb.blue  - loop_rgb.blue;
        double distance_rgb   = std::sqrt(std::pow(distance_red, 2) + std::pow(distance_green, 2) + std::pow(distance_blue, 2));
        printf("[%d] %f ", color_index, distance_rgb);
        if (nearest_index >= 0 && distance_rgb > nearest_distance) continue;
        nearest_index = color_index;
        nearest_distance = distance_rgb;
    }
    printf("\n");
    return nearest_index;
}