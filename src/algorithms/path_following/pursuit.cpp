#include "main.h"
#include "algorithms/path_following/pursuit.h"

Pursuit::Pursuit(float max_velocity) {

}
bool Pursuit::is_arrived() {
    return true;
}
void Pursuit::set_path(std::vector<Coordinates> input_path) {

}

void Pursuit::set_waypoint(std::vector<Waypoint> input_waypoint) {

}
ChassisVelocityPair Pursuit::step(RobotPosition position, bool reverse) {
    ChassisVelocityPair pair = ChassisVelocityPair();
    pair.left_v = 0;
    pair.right_v = 0;
    return pair;
}
