#include "algorithms/pure_pursuit/pure_pursuit.h"

using namespace okapi;

class Chassis {
    public:
        Chassis(struct Core* core);
        Chassis(struct Core* core, std::shared_ptr<Odom> odom);
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
        void faceCoordinate(float x, float y);
        void simpleMoveToPoint(float x, float y);
        void simpleMoveToPointBackwards(float x, float y);

        void followPath(std::vector<Coordinates> path, bool reverse = false);

        // position sensing functions
        float getLeftPosition();
        float getRightPosition();
        void tareSensors();

        // driver control functions
        float skim(float v);
        void cheezyDrive(float throttle, float turn);
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
        float Rd = 280;

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

        ADIEncoder* leftTW;
        ADIEncoder* rightTW;
        ADIEncoder* midTW;
        pros::IMU* imu1;
        pros::IMU* imu2;

        float exponential_filter(float input);
};