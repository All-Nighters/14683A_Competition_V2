#include "main.h"

/**
 * @brief Construct a new Chassis:: Chassis object
 * 
 */
Chassis::Chassis() {
    
}

/**
 * @brief Tare motor sensor readings
 * 
 */
void Chassis::tareSensors() {
    this->motor_lf->tarePosition();
    this->motor_lm->tarePosition();
    this->motor_lb->tarePosition();
    this->motor_rf->tarePosition();
    this->motor_rm->tarePosition();
    this->motor_rb->tarePosition();
}

/**
 * @brief Set brake mode for motors
 * 
 */
void Chassis::setBrakeMode(okapi::AbstractMotor::brakeMode brake_mode) {
    this->motor_lf->setBrakeMode(brake_mode);
    this->motor_lm->setBrakeMode(brake_mode);
    this->motor_lb->setBrakeMode(brake_mode);
    this->motor_rf->setBrakeMode(brake_mode);
    this->motor_rm->setBrakeMode(brake_mode);
    this->motor_rb->setBrakeMode(brake_mode);
}

/**
 * @brief Move the chassis motors with specified velocity
 * 
 * @param vel velocity
 */
void Chassis::moveVelocity(float vel) {
    this->motor_lf->moveVelocity(vel);
    this->motor_lm->moveVelocity(vel);
    this->motor_lb->moveVelocity(vel);
    this->motor_rf->moveVelocity(vel);
    this->motor_rm->moveVelocity(vel);
    this->motor_rb->moveVelocity(vel);
}

/**
 * @brief Move the chassis motors with specified velocity
 * 
 * @param left_vel left velocity
 * @param right_vel right velocity
 */
void Chassis::moveVelocity(float left_vel, float right_vel) {
    this->motor_lf->moveVelocity(left_vel);
    this->motor_lm->moveVelocity(left_vel);
    this->motor_lb->moveVelocity(left_vel);
    this->motor_rf->moveVelocity(right_vel);
    this->motor_rm->moveVelocity(right_vel);
    this->motor_rb->moveVelocity(right_vel);
}

/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param voltage 
 */
void Chassis::moveVoltage(float voltage) {
    this->motor_lf->moveVoltage(voltage);
    this->motor_lm->moveVoltage(voltage);
    this->motor_lb->moveVoltage(voltage);
    this->motor_rf->moveVoltage(voltage);
    this->motor_rm->moveVoltage(voltage);
    this->motor_rb->moveVoltage(voltage);
}


/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param left_volt left voltage
 * @param right_volt right voltage
 */
void Chassis::moveVoltage(float left_volt, float right_volt) {
    this->motor_lf->moveVoltage(left_volt);
    this->motor_lm->moveVoltage(left_volt);
    this->motor_lb->moveVoltage(left_volt);
    this->motor_rf->moveVoltage(right_volt);
    this->motor_rm->moveVoltage(right_volt);
    this->motor_rb->moveVoltage(right_volt);
}

/**
 * @brief Move the robot with a specified distance
 * 
 * @param pct forward distance in percents
 * @param max_voltage motor maximum voltage (-12000 to 12000)
 */
void Chassis::moveDistance(float pct, float max_voltage) {
    float target_distance = Constants::FIELD::FIELD_LENGTH * pct / 100;
    float revs;
    revs = target_distance / (M_PI*(CONFIG::ROBOT_PARAMETERS::WHEEL_DIAMETER.convert(meter))); // # of revolutions of wheels

    float targetAngle = revs * 360 + this->getLeftPosition();

    float targetFaceAngle = (this->imu1->get_rotation() + this->imu2->get_rotation()) / 2; 
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    int direction;

    if (revs < 0) {
        direction = -1;
    } else {
        direction = 1;
    }
    
    float prevErrorPosition = abs(targetAngle - this->getLeftPosition());
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
    RobotPosition robot_state = Odom::getState();
    float target_angle = angle;
    float prev_error = formatAngle(target_angle) - formatAngle(robot_state.theta);
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    while (abs(formatAngle(target_angle) - formatAngle(robot_state.theta)) >= 0.5 && pros::millis() - start_time <= timeout*1000) {
        robot_state = Odom::getState();

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
    return (this->motor_lf->getPosition() + this->motor_lm->getPosition()  + this->motor_lb->getPosition()) / 3.0;
}

/**
 * @brief Get right track motor position reading
 * 
 * @return right track motor position reading
 */
float Chassis::getRightPosition() {
    return (this->motor_rf->getPosition() + this->motor_rm->getPosition()  + this->motor_rb->getPosition()) / 3.0;
}

/**
 * @brief Cheezy driver control
 * 
 * @param throttle forward power
 * @param turn turn power
 */
void Chassis::cheezyDrive(float throttle, float turn) {
    float turn_vel = turn * abs(throttle);
    float forward_vel = throttle;

    this->moveVelocity(
        clamp(forward_vel + turn_vel, -200, 200), 
        clamp(forward_vel - turn_vel, -200, 200)
    );
}