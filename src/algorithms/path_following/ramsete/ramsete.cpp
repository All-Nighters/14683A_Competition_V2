#include "main.h"
#include "algorithms/path_following/pursuit.h"
#include "algorithms/path_following/ramsete/ramsete.h"

/**
 * @brief Construct a new Ramsete:: Ramsete object
 * 
 * @param max_velocity maximum drive velocity
 */
Ramsete::Ramsete(float max_velocity) : Pursuit(max_velocity) {
    this->max_velocity = max_velocity;
}

/**
 * @brief Construct a new Ramsete:: Ramsete object
 * 
 * @param input_path path to follow
 * @param max_velocity maximum velocity to drive
 */
Ramsete::Ramsete(std::vector<Waypoint> input_path, float max_velocity) : Pursuit(max_velocity) {
    this->path = input_path;
    this->max_velocity = max_velocity;
    this->arrived = false;
}

/**
 * @brief Check if the robot has reached the destination
 * 
 * @return true 
 * @return false 
 */
bool Ramsete::is_arrived() {
    return this->arrived;
}

/**
 * @brief Set the path to follow
 * 
 * @param input_path path to follow
 */
void Ramsete::set_path(std::vector<Waypoint> input_path) {
    this->path = input_path;
    this->arrived = false;
}

/**
 * @brief Set ramsete constants
 * 
 * @param b roughly a proportional term for the controller, must be > 0
 * @param zeta roughly a damping term (like the D term of a PID controller), Must be between 0 and 1
 */
void Ramsete::set_constants(float b, float zeta) {
    this->b = b;
    this->zeta = zeta;
}

/**
 * @brief Convert absolute coordinate to coordinate local to the robot
 * 
 * @param position robot position
 * @param point point to convert
 * @returns local coordinate 
 */
Waypoint Ramsete::absToLocal(RobotPosition position, Waypoint point) {
    Coordinates self_coordinate = Coordinates(position.x_pct, position.y_pct, position.theta);
    float xDist = point.get_x() - position.x_pct;
    float yDist = point.get_y() - position.y_pct; 

    // apply rotation matrix
    float newX = yDist*cos(position.theta*M_PI/180.0) - xDist*sin(position.theta*M_PI/180.0);
    float newY = yDist*sin(position.theta*M_PI/180.0) + xDist*cos(position.theta*M_PI/180.0);

    return Waypoint(newX, newY, position.theta, point.get_linear_vel(), point.get_ang_vel());
}

/**
 * @brief One step of the ramsete algorithm
 * 
 * @param position robot position
 * @param reverse whether the robot should pursue the path backwards
 * @returns left and right track velocity pair 
 */
ChassisVelocityPair Ramsete::step(RobotPosition position, bool reverse) {
    ChassisVelocityPair velocity_pair;
    if (!this->arrived && this->closest(position) < this->path.size()-1) {
        this->arrived = false;
        Waypoint look_ahead = this->path[this->closest(position)];
        Waypoint local_look_ahead = this->absToLocal(position, look_ahead);

        float e_x = local_look_ahead.get_x();
        float e_y = reverse ? -local_look_ahead.get_y() : local_look_ahead.get_y();
        float e_theta = 
            reverse ? Math::format_angle(180 + local_look_ahead.get_direction() - position.theta) 
                    : Math::format_angle(local_look_ahead.get_direction() - position.theta);

        float desired_linvel = look_ahead.get_linear_vel();
        float desired_angvel = std::fmin(std::fmax(0.1*abs(e_theta), 0), M_PI/2);
        float k = 2 * this->zeta * sqrt(desired_angvel * desired_angvel + this->b * desired_linvel * desired_linvel);

        float target_linvel = desired_linvel * cos(e_theta) + k * e_y;
        float target_angvel;

        if (e_theta != 0) {
            target_angvel = desired_angvel + k * e_theta + (this->b*desired_linvel*sin(e_theta)* e_x) / e_theta;
        }
        if (reverse) {
            velocity_pair.left_v = -(target_linvel + target_angvel);
            velocity_pair.right_v = -(target_linvel - target_angvel);
        } else {
            velocity_pair.left_v = target_linvel + target_angvel;
            velocity_pair.right_v = target_linvel - target_angvel;
        }
    }
    return velocity_pair;
}