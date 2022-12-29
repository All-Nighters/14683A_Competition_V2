#include <cmath>
#include "main.h"

Vision::Vision(struct Core* core) {
    this->core = core;
}

float Vision::get_direction(pros::vision_signature_s_t signature_pointer) {
    pros::Vision vision_sensor(1);
	vision_sensor.set_signature(1, &signature_pointer);
	auto object = vision_sensor.get_by_size(0);
    if (object.signature == 255) return 0.0f;
    int center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    float center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    float direction_radian = std::atan(center_x_percentage * std::tan(Math::degree_to_radian(61 / 2.0f)));
    return Math::radian_to_degree(direction_radian);
}