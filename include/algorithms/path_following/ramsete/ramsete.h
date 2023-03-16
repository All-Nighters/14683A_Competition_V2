#pragma once
#include "algorithms/path_following/pursuit.h"


class Ramsete : public Pursuit {
    public:
        Ramsete(float max_velocity = 300);
        Ramsete(std::vector<Waypoint> input_path, float max_velocity = 300);
        bool is_arrived() override;
        void set_waypoint(std::vector<Waypoint> input_waypoint) override;
        void set_constants(float b, float zeta);
        ChassisVelocityPair step(RobotPosition position, bool reverse = false) override;
    private:
        bool arrived;
        float look_ahead_radius;
        float max_velocity;
        float b = 0.1;
        float zeta = 0.01;

        std::vector<Waypoint> path;
        int closest(RobotPosition position);
        Waypoint absToLocal(RobotPosition position, Waypoint point);
};