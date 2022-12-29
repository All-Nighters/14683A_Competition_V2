#include "algorithms/pure_pursuit/pure_pursuit.h"

using namespace okapi;

class Chassis {
    public:
        Chassis(struct Core* core);
        Chassis(struct Core* core, Odom* odom);
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

        // position sensing functions
        float getLeftPosition();
        float getRightPosition();
        void tareSensors();

        // driver control functions
        float skim(float v);
        void cheezyDrive(float throttle, float turn);
    private:
        // translational PID constants
        float Tp;
        float Ti;
        float Td;

        // rotational PID constants
        float Rp;
        float Ri;
        float Rd;

        // directional PID constants (help driving straight)
        float Dp;
        float Di;
        float Dd;

        struct Core* core;
        Odom* odom;
        PurePursuit pure_pursuit;
        bool odom_enabled;
        AbstractMotor::gearset motor_gearset;
        float maximum_velocity;

        ADIEncoder* leftTW;
        ADIEncoder* rightTW;
        ADIEncoder* midTW;
        pros::IMU* imu1;
        pros::IMU* imu2;
};