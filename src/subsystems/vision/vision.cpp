#include <cmath>
#include "main.h"

Vision::Vision(struct Core* core) {
    this->core = core;
}

void Vision::set_signatures(std::vector<pros::vision_signature_s_t> signatures) {
    for (int signature_index = 0; signature_index < signatures.size(); signature_index++) {
        this->core->vision_goal->set_signature(signature_index + 1, &signatures[signature_index]);
    }
}

float Vision::get_direction() {
	auto object = this->core->vision_goal->get_by_size(0);
    if (object.signature == 255) return 0.0f;
    int center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    float center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    float direction_radian = std::atan(center_x_percentage * std::tan(Math::degree_to_radian(61 / 2.0f)));
    return Math::radian_to_degree(direction_radian);
}