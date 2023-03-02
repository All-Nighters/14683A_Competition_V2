#include "algorithms/pure_pursuit/pure_pursuit.h"

using namespace okapi;

class Chassis {
    public:
        Chassis(struct Core* core);
        Chassis(struct Core* core, std::shared_ptr<Odom> odom);
        Chassis(struct Core* core, std::shared_ptr<Odom> odom, float pursuit_Tp, float pursuit_Ti, float pursuit_Td);
        ~Chassis();
        void setBrakeMode(AbstractMotor::brakeMode brake_mode);

        // basic movement functions
        void moveVelocity(float vel);
        void moveVelocity(float left_vel, float right_vel);
        void moveVoltage(float voltage);
        void moveVoltage(float left_volt, float right_volt);

        // autonomous helper functions
        void moveDistance(float pct, float max_voltage = 12000);
        void turnAngle(float angle);
        void faceAngle(float angle);
        void faceCoordinate(float x, float y, float angle_offset = 0);
        void simpleMoveToPoint(float x, float y);
        void simpleMoveToPointBackwards(float x, float y, float max_voltage = 5000, bool turn_on_intake = false);

        void followPath(std::vector<Coordinates> path, bool reverse = false);

        // position sensing functions
        float getLeftPosition();
        float getRightPosition();
        void tareSensors();

        // driver control functions
        float skim(float v);
        void cheezyDrive(float throttle, float turn);
        void arcade(float throttle, float turn);
        void tank(float left, float right);
        void auto_aim();
        std::shared_ptr<Odom> odom;
    private:
        // translational PID constants
        float Tp = 5;
        float Ti = 0.015;
        float Td = 10;

        // rotational PID constants
        float Rp = 70;
        float Ri = 0.01;
        float Rd = 260;

        // directional PID constants
        float Dp = 200;
        float Di = 0.1;
        float Dd = 200;

        struct Core* core;
        PurePursuit pure_pursuit;
        std::unique_ptr<Vision> vision {nullptr};
        bool odom_enabled;
        AbstractMotor::gearset motor_gearset;
        float maximum_velocity;
        float total_error_autoaim;

        ADIEncoder* leftTW;
        ADIEncoder* rightTW;
        ADIEncoder* midTW;
        pros::IMU* imu1;
        pros::IMU* imu2;

        float exponential_filter(float input);
};