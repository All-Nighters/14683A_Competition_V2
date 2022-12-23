#include "algorithms/pure_pursuit/pure_pursuit.h"

/**
 * @brief Construct a new Pure Pursuit:: Pure Pursuit object
 * 
 * @param max_velocity maximum path following velocity
 */
PurePursuit::PurePursuit(float max_velocity) {
    this->max_velocity = max_velocity;
}

/**
 * @brief Construct a new Pure Pursuit:: Pure Pursuit object
 * 
 * @param input_path path to follow
 * @param max_velocity maximum path following velocity
 */
PurePursuit::PurePursuit(std::vector<Coordinates> input_path, float max_velocity) {
    this->max_velocity = max_velocity;
    this->arrived = false;
    for (Coordinates point : input_path) {
        this->path.push_back(point);
    }
}

/**
 * @brief Check if the robot has reached the destination
 * 
 * @return true 
 * @return false 
 */
bool PurePursuit::is_arrived() {
    return this->arrived;
}

/**
 * @brief Set the path to follow
 * 
 * @param input_path path to follow
 */
void PurePursuit::set_path(std::vector<Coordinates> input_path) {
    this->arrived = false;
    this->path.clear();
    for (Coordinates point : input_path) {
        this->path.push_back(point);
    }
}

/**
 * @brief Find the index number of the closest waypoint to the robot
 * 
 * @param position robot position
 * @returns index 
 */
int PurePursuit::closest(RobotPosition position) {
    float x_dist = this->path[0].get_x() - position.x_pct;
    float y_dist = this->path[0].get_y() - position.y_pct;
    float min_distance = sqrt(x_dist * x_dist + y_dist * y_dist);
    float min_idx = 0;
    for (int i = 1; i < path.size(); i++) {
        x_dist = this->path[i].get_x() - position.x_pct;
        y_dist = this->path[i].get_y() - position.y_pct;
        float dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        if (dist < min_distance) {
            min_distance = dist;
            min_idx = 0;
        }
    }
    return min_idx;
}

/**
 * @brief Determine the look ahead point
 * 
 * @param position robot position
 * @returns look ahead point 
 */
Coordinates PurePursuit::getLookAheadPoint(RobotPosition position) {
    float x_dist = this->path[this->path.size()-1].get_x() - position.x_pct;
    float y_dist = this->path[this->path.size()-1].get_y() - position.y_pct;
    float dist_to_end = sqrt(x_dist * x_dist + y_dist * y_dist);
    if (dist_to_end < this->look_ahead_radius) {
        return this->path[this->path.size()-1];
    }

    Coordinates self_coordinate = Coordinates(position.x_pct, position.y_pct, position.theta);
    Coordinates look_ahead_point = Coordinates(0, 0, 0);

    for (int i = 1; i < this->path.size(); i++) {
        Coordinates coord = this->path[i];
        Coordinates prevCoord = this->path[i-1];

        // printf("%f %f %f %f\n", coord.get_x(), coord.get_y(), selfCoordinate.get_x(), selfCoordinate.get_y());
        // if suitable distance is found
        if (coord.get_distance(self_coordinate) > this->look_ahead_radius && 
        prevCoord.get_distance(self_coordinate) < this->look_ahead_radius &&
        this->closest(position) < i) {

            // interpolation
            float prevX = prevCoord.get_x();
            float prevY = prevCoord.get_y();

            float currX = coord.get_x();
            float currY = coord.get_y();

            float minT = 0;
            float maxT = 1;

            float newX = prevX;
            float newY = prevY;

            int iterations = 10;

            // binary approximation
            for (int z = 0; z < iterations; z++) {
                float midT = (minT + maxT) / 2.0;
                newX = prevX * (1 - midT) + currX * midT;
                newY = prevY * (1 - midT) + currY * midT;

                look_ahead_point = Coordinates(newX, newY, 0);

                float distToSelf = look_ahead_point.get_distance(self_coordinate);

                if (distToSelf < this->look_ahead_radius) {
                    minT = midT;
                }
                else {
                    maxT = midT;
                }
            }
            return look_ahead_point;
        }
    }
    
    look_ahead_point = this->path[this->closest(position)];
    return look_ahead_point;

}

/**
 * @brief Convert absolute coordinate to coordinate local to the robot
 * 
 * @param position robot position
 * @param point point to convert
 * @returns local coordinate 
 */
Coordinates PurePursuit::absToLocal(RobotPosition position, Coordinates point) {
    Coordinates self_coordinate = Coordinates(position.x_pct, position.y_pct, position.theta);
    float xDist = point.get_x() - position.x_pct;
    float yDist = point.get_y() - position.y_pct; 

    // apply rotation matrix
    float newX = yDist*cos(position.theta*M_PI/180.0) - xDist*sin(position.theta*M_PI/180.0);
    float newY = yDist*sin(position.theta*M_PI/180.0) + xDist*cos(position.theta*M_PI/180.0);

    return Coordinates(newX, newY, position.theta);
}

/**
 * @brief One step of the pure pursuit algorithm
 * 
 * @param position robot position
 * @returns left and right track velocity pair 
 */
ChassisVelocityPair PurePursuit::step(RobotPosition position) {
    ChassisVelocityPair velocity_pair;
    if (this->closest(position) != this->path.size()-1) {
        this->arrived = false;
        Coordinates look_ahead = this->getLookAheadPoint(position);
        Coordinates local_look_ahead = this->absToLocal(position, look_ahead);

        float curv = (2*local_look_ahead.get_x()) / ((this->look_ahead_radius * this->look_ahead_radius));
        velocity_pair.left_v = this->max_velocity*(1+(curv*Constants::Robot::TRACK_LENGTH.convert(meter)/2));
        velocity_pair.left_v = this->max_velocity*(1-(curv*Constants::Robot::TRACK_LENGTH.convert(meter)/2));
    } else {
        velocity_pair.left_v = 0;
        velocity_pair.right_v = 0;
        this->arrived = true;
    }
    return velocity_pair;
}