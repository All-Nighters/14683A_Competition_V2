#include "main.h"


/**
 * @brief Construct a new Chassis:: Chassis object
 * 
 * @param core Core structure pointer
 */
Chassis::Chassis(struct Core* core) {
    this->core = core;
    this->pursuit_mode = PursuitMode::PID_PURE_PURSUIT;
    this->pursuit = new PurePursuit(600);
    this->odom_enabled = false;
    this->motor_gearset = AbstractMotor::gearset::blue;
    this->maximum_velocity = 600;
    this->imu1 = core->imu_first;
    this->imu2 = core->imu_second;
    this->vision = std::move(std::make_unique<Vision>(core));
    // signatures
    std::vector<pros::vision_signature_s_t> vision_signatures;
    vision_signatures.push_back(Constants::RED_SIG);
    vision_signatures.push_back(Constants::BLUE_SIG);
    this->vision->set_signatures(vision_signatures);
    printf("[Chassis]: Chassis created\n");
}

/**
 * @brief Construct a new Chassis:: Chassis object
 * 
 * @param core Core structure pointer
 * @param odom Odom structure pointer
 */
Chassis::Chassis(struct Core* core, std::shared_ptr<Odom> odom, PursuitMode pursuit_mode) {
    this->core = core;
    this->odom = odom;
    this->odom_enabled = true;
    this->motor_gearset = AbstractMotor::gearset::blue;
    this->maximum_velocity = 600;
    this->imu1 = core->imu_first;
    this->imu2 = core->imu_second;
    this->vision = std::move(std::make_unique<Vision>(core));
    this->pursuit_mode = pursuit_mode;
    if (pursuit_mode == PursuitMode::PID_PURE_PURSUIT) {
        this->pursuit = new PurePursuit(600);
    }  else {
        this->pursuit = new PurePursuit(600);
    }
    // signatures
    std::vector<pros::vision_signature_s_t> vision_signatures;
    vision_signatures.push_back(Constants::RED_SIG);
    vision_signatures.push_back(Constants::BLUE_SIG);
    this->vision->set_signatures(vision_signatures);
    printf("[Chassis]: Chassis created\n");
}

void Chassis::set_pursuit_pid_constant(float pursuit_Tp, float pursuit_Ti, float pursuit_Td) {
    if (this->pursuit_mode == PursuitMode::PID_PURE_PURSUIT) {
        this->pursuit = new PurePursuit(pursuit_Tp, pursuit_Ti, pursuit_Td, 600);
    } else {
        printf("[Chassis]: Nothing changed, pursuit mode not PID\n");
    }
}

/**
 * @brief Destroy the Chassis:: Chassis object
 * 
 */
Chassis::~Chassis() {
    this->moveVelocity(0);
    printf("[Chassis]: Chassis destroyed\n");
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
    this->core->chassis_left_middle ->moveVelocity(left_vel);
    this->core->chassis_left_back   ->moveVelocity(left_vel);
    this->core->chassis_right_front ->moveVelocity(right_vel);
    this->core->chassis_right_middle->moveVelocity(right_vel);
    this->core->chassis_right_back  ->moveVelocity(right_vel);
}

/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param voltage 
 */
void Chassis::moveVoltage(float voltage) {
    this->core->chassis_left_front  ->moveVoltage(voltage);
    this->core->chassis_left_middle ->moveVoltage(voltage);
    this->core->chassis_left_back   ->moveVoltage(voltage);
    this->core->chassis_right_front ->moveVoltage(voltage);
    this->core->chassis_right_middle->moveVoltage(voltage);
    this->core->chassis_right_back  ->moveVoltage(voltage);
}


/**
 * @brief Move the chassis motors with specified voltage
 * 
 * @param left_volt left voltage
 * @param right_volt right voltage
 */
void Chassis::moveVoltage(float left_volt, float right_volt) {
    this->core->chassis_left_front  ->moveVoltage(left_volt);
    this->core->chassis_left_middle ->moveVoltage(left_volt);
    this->core->chassis_left_back   ->moveVoltage(left_volt);
    this->core->chassis_right_front ->moveVoltage(right_volt);
    this->core->chassis_right_middle->moveVoltage(right_volt);
    this->core->chassis_right_back  ->moveVoltage(right_volt);
}

/**
 * @brief Move the robot with a specified distance
 * 
 * @param pct forward distance in percents
 * @param max_voltage motor maximum voltage (-12000 to 12000)
 */
void Chassis::moveDistance(float pct, float max_voltage) {
    float target_distance    = Constants::Field::FIELD_LENGTH * pct / 100;
    float revs               = target_distance / (M_PI*(Constants::Robot::TRACKING_WHEEL_DIAMETER.convert(meter))); // # of revolutions of wheels
    float targetAngle        = revs * 360 + this->getLeftPosition();
    float targetFaceAngle    = (this->imu1->get_rotation() + this->imu2->get_rotation()) / 2; 
    float start_time         = pros::millis();  
    int direction            = revs < 0 ? -1 : 1;
    float timeout            = 10; // maximum runtime in seconds
    
    float prev_error_position   = abs(targetAngle -  this->core->middle_tracking_wheel->get());
    float prev_control_output   = 0;
    float prev_face_angle_error = 0;
    float total_error_position  = 0;
    float total_error_facing    = 0;


    while (abs(targetAngle -  this->core->middle_tracking_wheel->get()) >= 10 && 
    pros::millis() - start_time <= timeout*1000) {

        float error_position = targetAngle - this->core->middle_tracking_wheel->get();
        total_error_position += error_position;

        float error_facing = targetFaceAngle - ((this->imu1->get_rotation() + this->imu2->get_rotation()) / 2);
        total_error_facing += error_facing;

        float deriv_position = error_position - prev_error_position;
        float deriv_facing = error_facing - prev_face_angle_error;


        float control_output = error_position * this->Tp + total_error_position * this->Ti + deriv_position * this->Td;
        float control_output_facing = error_facing * this->Dp + total_error_facing * this->Di + deriv_facing * this->Dd;

        float deriv_control_output = control_output - prev_control_output;
        if ((int)deriv_control_output / abs((int) deriv_control_output) == direction &&
            abs(deriv_control_output) > 400) {
            control_output = prev_control_output + direction * 400;
        }

        float control_output_Left  = std::fmax(std::fmin(control_output, max_voltage), -max_voltage) 
        + std::fmax(std::fmin(control_output_facing, max_voltage * 0.25), -max_voltage * 0.25);
        float control_output_Right = std::fmax(std::fmin(control_output, max_voltage), -max_voltage) 
        - std::fmax(std::fmin(control_output_facing, max_voltage * 0.25), -max_voltage * 0.25);

        if (abs(control_output_Left) < 3000) {
            control_output_Left = direction * 3000;
        }
        if (abs(control_output_Right) < 3000) {
            control_output_Right = direction * 3000;
        }
        prev_error_position = error_position;
        prev_control_output = control_output;
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
    float current_rotation = (this->imu1->get_rotation() + this->imu2->get_rotation())/2.0;
    float target_angle = current_rotation + angle;
    float prev_error = abs(angle);
    float total_error = 0;
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    while (abs(target_angle - current_rotation) >= 1 && pros::millis() - start_time <= timeout*1000) {
        current_rotation = (this->imu1->get_rotation() + this->imu2->get_rotation())/2.0;
        float error = target_angle - current_rotation;
        total_error += error;
        float deriv_error = error - prev_error;

        float control_output = Math::clamp(error * this->Rp + total_error * this->Ri + deriv_error * this->Rd, -12000, 12000);

        if (abs(control_output) < 3000) {
            control_output = control_output > 0 ? 3000 : -3000;
        }

        prev_error = error;

        this->moveVoltage(control_output, -control_output);
        pros::delay(20);
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
        printf("[Chassis]: Cannot run faceAngle because odometry is not enabled\n");
        return;
    }
    RobotPosition robot_state = this->odom->getState();
    float target_angle = angle;
    float prev_error = Math::format_angle(target_angle) - Math::format_angle(robot_state.theta);
    float start_time = pros::millis();  
    float timeout = 10; // maximum runtime in seconds

    while (abs(Math::format_angle(target_angle) - Math::format_angle(robot_state.theta)) >= 1 && pros::millis() - start_time <= timeout*1000) {
        robot_state = this->odom->getState();

        float error = Math::format_angle(target_angle) - Math::format_angle(robot_state.theta);
        printf("%[Chassis]: f\n", error);
        float deriv_error = error - prev_error;

        float control_output = Math::clamp(error * this->Rp + deriv_error * this->Rd, -12000, 12000);

        if (abs(control_output) < 3000) {
            control_output = control_output > 0 ? 3000 : -3000;
        }

        prev_error = error;

        this->moveVoltage(control_output, -control_output);

        pros::delay(20);
    }

    this->moveVoltage(0);
        
}

/**
 * @brief face the robot to a specific coordinate in percent
 * 
 * @param xPercent x coordinate of the target in percents
 * @param yPercent y coordinate of the target in percents
 */
void Chassis::faceCoordinate(float xPercent, float yPercent, float angle_offset) {
    if (!this->odom_enabled) { // odometry not configured
        printf("[Chassis]: Cannot run faceCoordinate because odometry is not enabled\n");
        return;
    }

    RobotPosition position = this->odom->getState();
    float xDist = xPercent - position.x_pct;
    float yDist = yPercent - position.y_pct;

    if (abs(xDist) < 0.1 && abs(yDist) < 0.1) {
        return;
    }

    float dist = sqrt(xDist*xDist + yDist*yDist);

    float relativeAngle;

    if (xDist > 0 && yDist > 0) { // first quadrant
        relativeAngle = atan(abs(yDist/xDist)) * 180 / M_PI;
    }
    else if (xDist > 0 && yDist < 0) { // second quadrant
        relativeAngle = -atan(abs(yDist/xDist)) * 180 / M_PI;
    }
    else if (xDist < 0 && yDist < 0) { // third quadrant
        relativeAngle = -180 + (atan(abs(yDist/xDist)) * 180 / M_PI);
    }
    else if (xDist < 0 && yDist > 0) { // fourth quadrant
        relativeAngle = 180 - (atan(abs(yDist/xDist)) * 180 / M_PI);
    }
    else if (xDist == 0 && yDist != 0) {
        relativeAngle = (yDist / abs(yDist))*90;
    }
    else if (xDist != 0 && yDist == 0) {
        relativeAngle = 0;
    } else {
        return;
    }

    float faceAngle;

    faceAngle = Math::format_angle(relativeAngle - position.theta + angle_offset);

    turnAngle(faceAngle);
}

/**
 * Move the robot to a specific coordinate in the field. The function may be inaccurate over long distances.
 * 
 * @param xPercent x coordinate of the target
 * @param yPercent y coordinate of the target
 */
void Chassis::simpleMoveToPoint(float xPercent, float yPercent) {
    RobotPosition position = this->odom->getState();
    float xDist = xPercent - position.x_pct;
    float yDist = yPercent - position.y_pct;
    float dist = sqrt(xDist*xDist + yDist*yDist);

    this->faceCoordinate(xPercent, yPercent);
    position = this->odom->getState();
    xDist = xPercent - position.x_pct;
    yDist = yPercent - position.y_pct;
    dist = sqrt(xDist*xDist + yDist*yDist);
    this->moveDistance(dist);
}

/**
 * Move the robot backwards to a specific coordinate in the field. The function may be inaccurate over long distances.
 * 
 * @param xPercent x coordinate of the target
 * @param yPercent y coordinate of the target
 */
void Chassis::simpleMoveToPointBackwards(float xPercent, float yPercent, float max_voltage, bool turn_on_intake) {
    RobotPosition position = this->odom->getState();
    float xDist = xPercent - position.x_pct;
    float yDist = yPercent - position.y_pct;
    float dist = sqrt(xDist*xDist + yDist*yDist);

    this->faceCoordinate(position.x_pct - xDist, position.y_pct - yDist);
    position = this->odom->getState();
    xDist = xPercent - position.x_pct;
    yDist = yPercent - position.y_pct;
    dist = sqrt(xDist*xDist + yDist*yDist);
    if (turn_on_intake && this->core->catapult_load_sensor->get_value() == 1) {
        this->core->intake->moveVoltage(-10000);
    }
    this->moveDistance(-dist, max_voltage);
    if (turn_on_intake) {
        this->core->intake->moveVoltage(0);
    }
    

}

/**
 * @brief Follow a path
 * 
 * @param path path
 * @param reverse whether the robot should move backwards to pursue the path
 * @param enable_disk_pursuit whether the robot should pursue nearby disks
 */
void Chassis::followPath(std::vector<Coordinates> path, bool reverse) {
    this->pursuit->set_path(path);
    
    while (!this->pursuit->is_arrived()) {
        RobotPosition position = this->odom->getState();
        ChassisVelocityPair velocity_pair;
        velocity_pair = this->pursuit->step(position, reverse);
        this->moveVelocity(velocity_pair.left_v, velocity_pair.right_v);
        pros::delay(10);
    }
    this->moveVelocity(0);

}

/**
 * @brief Get left track motor position reading
 * 
 * @return left track motor position reading
 */
float Chassis::getLeftPosition() {
    return (this->core->chassis_left_front->getPosition() + this->core->chassis_left_middle->getPosition() + this->core->chassis_left_back->getPosition()) / 3 * 36/60 / 77*90;
}

/**
 * @brief Get right track motor position reading
 * 
 * @return right track motor position reading
 */
float Chassis::getRightPosition() {
    return (this->core->chassis_right_front->getPosition() + this->core->chassis_right_middle->getPosition() + this->core->chassis_right_back->getPosition()) / 3 * 36/60 / 77*90;
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

float Chassis::exponential_filter(float input) {
    return Math::clamp((1.2*pow(1.045315, 100*input) - 1.2 + 0.3*input) / 100, 0, 1);
}
/**
 * @brief Cheezy driver control
 * 
 * @param throttle forward power (-1 to 1)
 * @param turn turn power (-1 to 1)
 */
void Chassis::cheezyDrive(float throttle, float turn) {

    // apply exponential filter on joystick values
    if (throttle > 0) {
        throttle = this->exponential_filter(throttle);
    } else {
        throttle = -this->exponential_filter(-throttle);
    }

    if (turn > 0) {
        turn = this->exponential_filter(turn);
    } else {
        turn = -this->exponential_filter(-turn);
    }

    if (abs(turn) > 0.5) {
        turn = turn > 0 ? 0.5 : -0.5;
    }

    float t_left = throttle + turn;
    float t_right = throttle - turn;

    float left = t_left + skim(t_right);
    float right = t_right + skim(t_left);

    if (abs(left) < 0.01 && abs(right) < 0.01) {
        this->moveVelocity(0);
    } else {
        this->moveVelocity(
            Math::clamp(left * this->maximum_velocity, -this->maximum_velocity, this->maximum_velocity), 
            Math::clamp(right * this->maximum_velocity, -this->maximum_velocity, this->maximum_velocity)
        );
    } 
}

/**
 * @brief Arcade driver control
 * 
 * @param throttle forward power (-1 to 1)
 * @param turn turn power (-1 to 1)
 */
void Chassis::arcade(float throttle, float turn) {
    // apply exponential filter on joystick values
    if (throttle > 0) {
        throttle = this->exponential_filter(throttle);
    } else {
        throttle = -this->exponential_filter(-throttle);
    }

    if (turn > 0) {
        turn = this->exponential_filter(turn);
    } else {
        turn = -this->exponential_filter(-turn);
    }
    
    float left = this->maximum_velocity * Math::clamp(throttle + turn, -1, 1);
    float right = this->maximum_velocity * Math::clamp(throttle - turn, -1, 1);
    this->moveVelocity(left, right);
}

/**
 * @brief Tank driver control
 * 
 * @param left left power (-1 to 1)
 * @param right right power (-1 to 1)
 */
void Chassis::tank(float left, float right) {
    // apply exponential filter on joystick values
    if (left > 0) {
        left = this->exponential_filter(left);
    } else {
        left = -this->exponential_filter(-left);
    }

    if (right > 0) {
        right = this->exponential_filter(right);
    } else {
        right = -this->exponential_filter(-right);
    }
    this->moveVelocity(this->maximum_velocity * left, this->maximum_velocity * right);
}

void Chassis::auto_aim() {
    float angle = this->vision->get_direction();

    float current_rotation = (this->imu1->get_rotation() + this->imu2->get_rotation())/2.0;
    float target_angle = current_rotation + angle;
    float prev_error = angle;

    float error = target_angle - current_rotation;
    total_error_autoaim += error;
    float deriv_error = error - prev_error;

    float control_output = Math::clamp(error * 500 + total_error_autoaim * 3 + deriv_error * 900, -12000, 12000);
    if (abs(control_output) < 1000) {
        if (control_output < 0) {
            control_output = -1000;
        } else {
            control_output = 1000;
        }
    }

    prev_error = error;

    this->moveVoltage(control_output, -control_output);
}