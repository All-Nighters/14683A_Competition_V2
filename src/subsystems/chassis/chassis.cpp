#include "main.h"

/**
 * @brief Construct a new Chassis:: Chassis object
 * 
 */
Chassis::Chassis(struct Core* core) {
    this->core = core;
    this->odom_enabled = false;
    this->motor_gearset = AbstractMotor::gearset::blue;
    this->maximum_velocity = 600;
}

/**
 * @brief Construct a new Chassis:: Chassis object
 * 
 */
Chassis::Chassis(struct Core* core, Odom* odom) {
    this->core = core;
    this->odom = odom;
    this->odom_enabled = true;
    this->motor_gearset = AbstractMotor::gearset::blue;
    this->maximum_velocity = 600;
}

/**
 * @brief Tare motor sensor readings
 * 
 */
void Chassis::tareSensors() {
    this->core->chassis_left_front->tarePosition();
    this->core->chassis_left_middle->tarePosition();
    this->core->chassis_left_back->tarePosition();
    this->core->chassis_right_front->tarePosition();
    this->core->chassis_right_middle->tarePosition();
    this->core->chassis_right_back->tarePosition();
}

/**
 * @brief Set brake mode for motors
 * 
 */
void Chassis::setBrakeMode(okapi::AbstractMotor::brakeMode brake_mode) {
    this->core->chassis_left_front->setBrakeMode(brake_mode);
    this->core->chassis_left_middle->setBrakeMode(brake_mode);
    this->core->chassis_left_back->setBrakeMode(brake_mode);
    this->core->chassis_right_front->setBrakeMode(brake_mode);
    this->core->chassis_right_middle->setBrakeMode(brake_mode);
    this->core->chassis_right_back->setBrakeMode(brake_mode);
}

/**
 * @brief Move the chassis motors with specified velocity
 * 
 * @param vel velocity
 */
void Chassis::moveVelocity(float vel) {
    vel = vel / 600.0 * 200;
    this->core->chassis_left_front  ->moveVelocity(vel);
    this->core->chassis_left_middle ->moveVelocity(vel);
    this->core->chassis_left_back   ->moveVelocity(vel);
    this->core->chassis_right_front ->moveVelocity(vel);
    this->core->chassis_right_middle->moveVelocity(vel);
    this->core->chassis_right_back  ->moveVelocity(vel);
}

/**
 * @brief Move the chassis motors with specified velocity
 * 
 * @param left_vel left velocity
 * @param right_vel right velocity
 */
void Chassis::moveVelocity(float left_vel, float right_vel) {
    left_vel = left_vel / 600.0 * 200;
    right_vel = right_vel / 600.0 * 200;
    this->core->chassis_left_front  ->moveVelocity(left_vel);
    // this->core->chassis_left_middle ->moveVelocity(left_vel);
    // this->core->chassis_left_back   ->moveVelocity(left_vel);
    this->core->chassis_right_front ->moveVelocity(right_vel);
    // this->core->chassis_right_middle->moveVelocity(right_vel);
    // this->core->chassis_right_back  ->moveVelocity(right_vel);
}

/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param voltage 
 */
void Chassis::moveVoltage(float voltage) {
    this->core->chassis_left_front  ->moveVelocity(voltage);
    this->core->chassis_left_middle ->moveVelocity(voltage);
    this->core->chassis_left_back   ->moveVelocity(voltage);
    this->core->chassis_right_front ->moveVelocity(voltage);
    this->core->chassis_right_middle->moveVelocity(voltage);
    this->core->chassis_right_back  ->moveVelocity(voltage);
}


/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param left_volt left voltage
 * @param right_volt right voltage
 */
void Chassis::moveVoltage(float left_volt, float right_volt) {
    this->core->chassis_left_front  ->moveVelocity(left_volt);
    this->core->chassis_left_middle ->moveVelocity(left_volt);
    this->core->chassis_left_back   ->moveVelocity(left_volt);
    this->core->chassis_right_front ->moveVelocity(right_volt);
    this->core->chassis_right_middle->moveVelocity(right_volt);
    this->core->chassis_right_back  ->moveVelocity(right_volt);
}

/**
 * @brief Move the robot with a specified distance
 * 
 * @param pct forward distance in percents
 * @param max_voltage motor maximum voltage (-12000 to 12000)
 */
void Chassis::moveDistance(float pct, float max_voltage) {
    float target_distance    = Constants::Field::FIELD_LENGTH * pct / 100;
    float revs               = target_distance / (M_PI*(Constants::Robot::WHEEL_DIAMETER.convert(meter))); // # of revolutions of wheels
    float targetAngle        = revs * 360 + this->getLeftPosition();
    float targetFaceAngle    = (this->imu1->get_rotation() + this->imu2->get_rotation()) / 2; 
    float start_time         = pros::millis();  
    int direction            = revs < 0 ? -1 : 1;
    float timeout            = 10; // maximum runtime in seconds
    
    float prevErrorPosition  = abs(targetAngle - this->getLeftPosition());
    float prevFaceAngleError = 0;

    while (abs(targetAngle - this->getLeftPosition()) >= 10 && 
    pros::millis() - start_time <= timeout*1000) {

        float error_position = abs(targetAngle - this->getLeftPosition());

        prevErrorPosition = abs(targetAngle - this->getLeftPosition());
        float error_Facing = targetFaceAngle- ((this->imu1->get_rotation() + this->imu2->get_rotation()) / 2);


        float deriv_position = error_position - prevErrorPosition;
        float deriv_Facing = error_Facing - prevFaceAngleError;


        float control_output = error_position * this->Tp + deriv_position * this->Td;
        float control_output_Facing = error_Facing * this->Dp + deriv_Facing * this->Dd;

        float control_output_Left = direction * std::fmax(std::fmin(control_output, max_voltage), -max_voltage) + std::fmax(std::fmin(control_output_Facing, max_voltage * 0.25), -max_voltage * 0.25);
        float control_output_Right = direction * std::fmax(std::fmin(control_output, max_voltage), -max_voltage) - std::fmax(std::fmin(control_output_Facing, max_voltage * 0.25), -max_voltage * 0.25);

        if (abs(control_output_Left) < 2000 && direction < 0) {
            control_output_Left = -2000;
            control_output_Right = -2000;
        }
        else if (abs(control_output_Left) < 2000 && direction > 0) {
            control_output_Left = 2000;
            control_output_Right = 2000;
        }
        prevErrorPosition = error_position;
        this->moveVoltage(control_output_Left, control_output_Right);
        pros::delay(20);
    }
    this->moveVoltage(0);
}

/**
 * @brief Turn the robot with a specified angle
 * 
 * @param angle angle in degrees
 */
void Chassis::turnAngle(float angle) {
    float current_rotation = (this->imu1->get_rotation() + this->imu2->get_rotation()) / 2;
    float target_angle = current_rotation + angle;
    float prev_error = abs(angle);
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    while (abs(target_angle - current_rotation) >= 1 && pros::millis() - start_time <= timeout*1000) {
        float error = target_angle - current_rotation;
        float deriv_error = error - prev_error;

        float control_output = clamp(error * this->Rp + deriv_error * this->Rd, -12000, 12000);

        if (abs(control_output) < 2000) {
            control_output = control_output > 0 ? 2000 : -2000;
        }

        prev_error = error;

        this->moveVoltage(control_output, -control_output);
        pros::delay(20);
        current_rotation = (this->imu1->get_rotation() + this->imu2->get_rotation()) / 2;
    }

    this->moveVoltage(0);
        
}

/**
 * @brief PID controlling robot's absolute direction
 * 
 * @param angle desired absolute angle facing
 */

void Chassis::faceAngle(float angle) {
    if (!this->odom_enabled) { // odometry not configured
        printf("Cannot run faceAngle because odometry is not enabled\n");
        return;
    }
    RobotPosition robot_state = this->odom->getState();
    float target_angle = angle;
    float prev_error = formatAngle(target_angle) - formatAngle(robot_state.theta);
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    while (abs(formatAngle(target_angle) - formatAngle(robot_state.theta)) >= 0.5 && pros::millis() - start_time <= timeout*1000) {
        robot_state = this->odom->getState();

        float error = formatAngle(target_angle) - formatAngle(robot_state.theta);
        float deriv_error = error - prev_error;

        float control_output = clamp(error * this->Rp + deriv_error * this->Rd, -12000, 12000);

        if (abs(control_output) < 2000) {
            control_output = control_output > 0 ? 2000 : -2000;
        }

        prev_error = error;

        this->moveVoltage(control_output, -control_output);

        pros::delay(20);
    }

    this->moveVoltage(0);
        
}

/**
 * @brief Get left track motor position reading
 * 
 * @return left track motor position reading
 */
float Chassis::getLeftPosition() {
    return (this->core->chassis_left_front->getPosition() + this->core->chassis_left_middle->getPosition()  + this->core->chassis_left_back->getPosition());
}

/**
 * @brief Get right track motor position reading
 * 
 * @return right track motor position reading
 */
float Chassis::getRightPosition() {
    return (this->core->chassis_right_front->getPosition() + this->core->chassis_right_middle->getPosition()  + this->core->chassis_right_back->getPosition());
}

float Chassis::skim(float v) {
    float gain = 1;
    if (v > 1.0) {
        return -((v - 1.0) * gain);
    }   
    else if (v < -1.0) {
        return -((v + 1.0) * gain);
    }
        
    return 0;
}

/**
 * @brief Cheezy driver control
 * 
 * @param throttle forward power (-1 to 1)
 * @param turn turn power (-1 to 1)
 */
void Chassis::cheezyDrive(float throttle, float turn) {
    float t_left = throttle + turn;
    float t_right = throttle - turn;

    float left = t_left + skim(t_right);
    float right = t_right + skim(t_left);
    this->moveVelocity(
        clamp(left * this->maximum_velocity, -this->maximum_velocity, this->maximum_velocity), 
        clamp(right * this->maximum_velocity, -this->maximum_velocity, this->maximum_velocity)
    );
}