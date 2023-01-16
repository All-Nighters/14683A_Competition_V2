#include "main.h"

/**
 * @brief Construct a new Disk Pursuit:: Disk Pursuit object
 * 
 * @param core core structure pointer
 * @param max_velocity maximum pursuit velocity
 */
DiskPursuit::DiskPursuit(struct Core* core, float forward_velocity, float max_velocity) {
    this->core = core;
    this->forward_velocity = forward_velocity;
    this->max_velocity = max_velocity;
}

/**
 * @brief Set disk color signature
 * 
 * @param signature color signature pointer
 */
void DiskPursuit::set_signature(pros::vision_signature_s_t* signature) {
    this->core->vision_intake->set_signature(1, signature);
}

pros::vision_object_s_t DiskPursuit::get_closest_disk() {
    return this->core->vision_goal->get_by_size(0);
}
/**
 * @brief Get disk distance to the robot
 * 
 * @param object identified disk object
 * @returns distance in meters
 */
float DiskPursuit::get_disk_distance(pros::vision_object_s_t object) {
    int center_y = object.top_coord + (object.width / 2) - (VISION_FOV_HEIGHT / 2);
    int center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    float center_y_percentage = center_y / (VISION_FOV_WIDTH / 2.0f);
    float center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    float direction_radian = std::atan(center_y_percentage * std::tan(Math::degree_to_radian(this->max_vertical_angle / 2.0f)));
    float theta_diff = Math::degree_to_radian(this->max_vertical_angle) / 2.0 + direction_radian;
    float distance = this->sensor_height / std::tan(theta_diff + this->theta_initial);
    return sqrt(pow(distance, 2) + pow(center_x_percentage * distance * std::tan(this->max_horizontal_angle / 2.0), 2));
}

/**
 * @brief Get disk direction to the robot
 * 
 * @param object identified disk object
 * @returns angle in degrees 
 */
float DiskPursuit::get_disk_direction(pros::vision_object_s_t object) {
    int center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    float center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    float direction_radian = std::atan(center_x_percentage * std::tan(Math::degree_to_radian(61 / 2.0f)));
    return Math::radian_to_degree(direction_radian);
}

/**
 * @brief One step of PID pursuit algorithm
 * 
 * @returns left and right drivetrain velocity 
 */
ChassisVelocityPair DiskPursuit::step(bool reverse) {
    ChassisVelocityPair velocity_pair;
    pros::vision_object_s_t object = this->get_closest_disk();
    if (object.signature == 255) return velocity_pair;
    
    float error = this->get_disk_direction(object);
    float deriv_error = error - this->prev_error_direction;
    this->total_error_direction += error;
    float control_output = this->Kp * error + this->Ki * total_error_direction + this->Kd * deriv_error;
    prev_error_direction = error;

    if (reverse) {
        velocity_pair.left_v = Math::clamp(-(this->forward_velocity - control_output), -this->max_velocity, this->max_velocity);
        velocity_pair.right_v = Math::clamp(-(this->forward_velocity + control_output), -this->max_velocity, this->max_velocity);
    } else {
        velocity_pair.left_v = Math::clamp(this->forward_velocity + control_output, -this->max_velocity, this->max_velocity);
        velocity_pair.right_v = Math::clamp(this->forward_velocity - control_output, -this->max_velocity, this->max_velocity);
    }
    
    return velocity_pair;
}