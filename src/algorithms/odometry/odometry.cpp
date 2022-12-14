#define DEBUG true
#include "main.h"

Odom::Odom() {
    this->WHEEL_RADIUS = CONFIG::ROBOT_PARAMETERS::WHEEL_DIAMETER.convert(meter) / 2;
    this->LTrackRadius = CONFIG::ROBOT_PARAMETERS::TRACK_LENGTH.convert(meter) / 2;
    this->RTrackRadius = CONFIG::ROBOT_PARAMETERS::TRACK_LENGTH.convert(meter) / 2;
    this->STrackRadius = CONFIG::ROBOT_PARAMETERS::MIDDLE_ENCODER_DISTANCE.convert(meter) / 2;


    this->reset_variables();

    pros::Task loop(this->start_odom, this, "Odom");
}

void Odom::reset_variables() {
    this->LPos = 0;
    this->RPos = 0;
    this->SPos = 0;

    this->LPrevPos = 0;
    this->RPrevPos = 0;
    this->SPrevPos = 0;

    this->deltaDistL = 0;
    this->deltaDistR = 0;
    this->deltaDistS = 0;

    this->totalDeltaDistL = 0;
    this->totalDeltaDistR = 0;

    this->currentAbsoluteOrientation = THETA_START;
    this->previousTheta = THETA_START;
    this->deltaTheta = 0;
    this->avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

    this->deltaXLocal = 0;
    this->deltaYLocal = 0;
    this->deltaXGlobal = 0;
    this->deltaYGlobal = 0;

    this->xPosGlobal = X_START;
    this->yPosGlobal = Y_START;
}

void Odom::start_odom(void* iparam) {
    if(iparam){
        Odom* that = static_cast<Odom*>(iparam);
        that->position_tracking();
    }
}

/**
 * @brief Get the robot's position
 * 
 * @return RobotPosition structure
 */
RobotPosition Odom::getState() {
    return this->position;
}

/**
 * @brief Set the robot's position
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param angle angle
 */
void Odom::setState(QLength x, QLength y, QAngle angle) {
    // clear variables
    // this->chassis->tareSensors();
    this->X_START = x.convert(meter);
    this->Y_START = y.convert(meter);
    this->THETA_START = angle.convert(radian);

    this->reset_variables();
    // save odom state
    this->position.x_meter = xPosGlobal;
    this->position.y_meter = yPosGlobal;
    this->position.x_pct = xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    this->position.y_pct = yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    this->position.theta = currentAbsoluteOrientation;


}

/**
 * @brief Set the robot's position
 * 
 * @param x x coordinate in percents
 * @param y y coordinate in percents
 * @param angle angle in degrees
 */
void Odom::setState(float x, float y, float angle) {
    // clear variables
    // this->chassis->tareSensors();
    this->X_START = x / 100.0 * Constants::FIELD::FIELD_LENGTH;
    this->Y_START = y / 100.0 * Constants::FIELD::FIELD_LENGTH;
    this->THETA_START = angle * pi / 180;

    this->reset_variables();

    // save odom state
    this->position.x_meter = xPosGlobal;
    this->position.y_meter = yPosGlobal;
    this->position.x_pct = xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    this->position.y_pct = yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    this->position.theta = currentAbsoluteOrientation;
}

/**
 * @brief Position tracking loop. 
 * 
 * It should be ran with a separate task
 */
void Odom::position_tracking() {
    printf("Position tracking started\n");
    while(1) {
        // get position values
        switch (this->odometry_mode) {
            case OdomMode::MOTOR_IMU:
                // this->LPos = this->chassis->getLeftPosition();
                // this->RPos = this->chassis->getRightPosition();
                if (DEBUG) {
                    printf("Encoder: L=%f, R=%f\n", this->LPos, this->RPos);
                }
                break;
            case OdomMode::MOTOR_MIDTW_IMU:
                // this->LPos = this->chassis->getLeftPosition();
                // this->RPos = this->chassis->getRightPosition();
                this->SPos = this->midTW->get();
                if (DEBUG) {
                    printf("Encoder: L=%f, R=%f, S=%f\n", this->LPos, this->RPos, this->SPos);
                }
                break;
            case OdomMode::LEFTTW_MIDTW_IMU:
                this->LPos = this->leftTW->get();
                this->SPos = this->midTW->get();
                if (DEBUG) {
                    printf("Encoder: L=%f, S=%f\n", this->LPos, this->SPos);
                }
                break;
            case OdomMode::RIGHTTW_MIDTW_IMU:
                this->RPos = this->rightTW->get();
                this->SPos = this->midTW->get();
                if (DEBUG) {
                    printf("Encoder: R=%f, S=%f\n", this->RPos, this->SPos);
                }
                break;
        }
        

        //Calculate distance traveled by tracking each wheel (meters)
        //Converts degrees to radians
        this->deltaDistL = ((this->LPos - this->LPrevPos) * M_PI / 180) * this->WHEEL_RADIUS;
        this->deltaDistR = ((this->RPos - this->RPrevPos) * M_PI / 180) * this->WHEEL_RADIUS;
        this->deltaDistS = ((this->SPos - this->SPrevPos) * M_PI / 180) * this->WHEEL_RADIUS;

        //Update previous values to be used next loop (DEGREES)
        this->LPrevPos = this->LPos;
        this->RPrevPos = this->RPos;
        this->SPrevPos = this->SPos;

        //Total change in each of the L and R encoders since last reset (meters)
        //These are used to calculate the absolute orientation of the bot
        this->totalDeltaDistL += this->deltaDistL;
        this->totalDeltaDistR += this->deltaDistR;

        //Calculate the current absolute orientation (RADIANS)

        this->currentAbsoluteOrientation = this->THETA_START + ((this->imu1->get_rotation() + this->imu2->get_rotation()) / 2) * M_PI / 180.0;
        if (isinf(this->currentAbsoluteOrientation)) {
            this->currentAbsoluteOrientation = 0;
        }

        //Calculate the change in the angle of the bot (RADIANS)
        this->deltaTheta = this->currentAbsoluteOrientation - this->previousTheta;

        //Update the previous Theta value (RADIANS)  
        this->previousTheta = this->currentAbsoluteOrientation;

        //If we didn't turn, then we only translated
        if(this->deltaTheta == 0) {
            this->deltaXLocal = this->deltaDistS;
            this->deltaYLocal = this->deltaDistL;
        }
        //Else, caluclate the new local position
        else {
        //Calculate the changes in the X and Y values (meters)

        // The calculation assumes that the middle tracking wheel is at the front of the machine
        this->deltaXLocal = 2 * sin(this->deltaTheta / 2.0) * ((this->deltaDistS / this->deltaTheta) - this->STrackRadius);
        this->deltaYLocal = 2 * sin(this->deltaTheta / 2.0) * ((this->deltaDistR / this->deltaTheta) + this->RTrackRadius);
        }

        //The average angle of the robot during it's arc (RADIANS)
        this->avgThetaForArc = this->currentAbsoluteOrientation - (this->deltaTheta / 2);

        this->deltaXGlobal = (this->deltaYLocal * cos(this->avgThetaForArc)) - (this->deltaXLocal * sin(this->avgThetaForArc));
        this->deltaYGlobal = (this->deltaYLocal * sin(this->avgThetaForArc)) + (this->deltaXLocal * cos(this->avgThetaForArc));

        //Update global positions
        this->xPosGlobal += this->deltaXGlobal;
        this->yPosGlobal += this->deltaYGlobal;

        this->position.x_meter = this->xPosGlobal;
        this->position.y_meter = this->yPosGlobal;
        this->position.x_pct = this->xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
        this->position.y_pct = this->yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
        this->position.theta = this->currentAbsoluteOrientation;

        pros::delay(10);

    }
}