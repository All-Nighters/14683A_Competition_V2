#pragma once

class Pursuit {
    public:
        Pursuit(float max_velocity);
        virtual bool is_arrived();
        virtual void set_path(std::vector<Coordinates> input_path);
        virtual ChassisVelocityPair step(RobotPosition position, bool reverse = false);
};