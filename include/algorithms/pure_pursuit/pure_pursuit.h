#pragma once

typedef struct ChassisVelocityPair {
    float left_v;
    float right_v;
} ChassisVelocityPair;

class PurePursuit {
    public:
        PurePursuit(float max_velocity = 300);
        PurePursuit(std::vector<Coordinates> input_path, float max_velocity = 300);
        bool is_arrived();
        void set_path(std::vector<Coordinates> input_path);
        ChassisVelocityPair step(RobotPosition position, bool reverse = false);
    private:
        bool arrived;
        float look_ahead_radius;
        float max_velocity;

        // PID terms
        float kP_displacement;
        float kP_rotation;
        float kI_displacement;
        float kI_rotation;
        float kD_displacement;
        float kD_rotation;

        float prev_displacement_error;
        float prev_rotation_error;
        float total_displacement_error;
        float total_rotation_error;
        std::vector<Coordinates> path;
        void init();
        void reset_error();
        int closest(RobotPosition position);
        Coordinates getLookAheadPoint(RobotPosition position);
        Coordinates absToLocal(RobotPosition position, Coordinates point);
};