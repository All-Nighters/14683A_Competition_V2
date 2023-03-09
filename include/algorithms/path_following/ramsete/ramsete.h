#pragma once
#include "algorithms/path_following/pursuit.h"


class Ramsete : public Pursuit {
    public:
        Ramsete(float max_velocity = 300);
        Ramsete(std::vector<Waypoint> input_path, float max_velocity = 300);
        bool is_arrived() override;
        void set_path(std::vector<Waypoint> input_path);
        void set_constants(float b, float zeta);
        ChassisVelocityPair step(RobotPosition position, bool reverse = false) override;
    private:
        bool arrived;
        float look_ahead_radius;
        float max_velocity;
        float b = 0.5;
        float zeta = 1;

        std::vector<Waypoint> path;
        int closest(RobotPosition position);
        Waypoint absToLocal(RobotPosition position, Waypoint point);
};