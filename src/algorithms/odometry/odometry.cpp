#define DEBUG true
#include "main.h"

Odom::Odom() {
    Odom::WHEEL_RADIUS = CONFIG::ROBOT_PARAMETERS::WHEEL_DIAMETER.convert(meter) / 2;
    Odom::LTrackRadius = CONFIG::ROBOT_PARAMETERS::TRACK_LENGTH.convert(meter) / 2;
    Odom::RTrackRadius = CONFIG::ROBOT_PARAMETERS::TRACK_LENGTH.convert(meter) / 2;
    Odom::STrackRadius = CONFIG::ROBOT_PARAMETERS::MIDDLE_ENCODER_DISTANCE.convert(meter) / 2;


    Odom::reset_variables();

    pros::Task loop(Odom::position_tracking);
}

void Odom::reset_variables() {
    Odom::LPos = 0;
    Odom::RPos = 0;
    Odom::SPos = 0;

    Odom::LPrevPos = 0;
    Odom::RPrevPos = 0;
    Odom::SPrevPos = 0;

    Odom::deltaDistL = 0;
    Odom::deltaDistR = 0;
    Odom::deltaDistS = 0;

    Odom::totalDeltaDistL = 0;
    Odom::totalDeltaDistR = 0;

    Odom::currentAbsoluteOrientation = THETA_START;
    Odom::previousTheta = THETA_START;
    Odom::deltaTheta = 0;
    Odom::avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

    Odom::deltaXLocal = 0;
    Odom::deltaYLocal = 0;
    Odom::deltaXGlobal = 0;
    Odom::deltaYGlobal = 0;

    Odom::xPosGlobal = X_START;
    Odom::yPosGlobal = Y_START;
}

/**
 * @brief Get the robot's position
 * 
 * @return RobotPosition structure
 */
RobotPosition Odom::getState() {
    return Odom::position;
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
    // Odom::chassis->tareSensors();
    Odom::X_START = x.convert(meter);
    Odom::Y_START = y.convert(meter);
    Odom::THETA_START = angle.convert(radian);

    Odom::reset_variables();
    // save odom state
    Odom::position.x_meter = xPosGlobal;
    Odom::position.y_meter = yPosGlobal;
    Odom::position.x_pct = xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    Odom::position.y_pct = yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    Odom::position.theta = currentAbsoluteOrientation;


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
    // Odom::chassis->tareSensors();
    Odom::X_START = x / 100.0 * Constants::FIELD::FIELD_LENGTH;
    Odom::Y_START = y / 100.0 * Constants::FIELD::FIELD_LENGTH;
    Odom::THETA_START = angle * pi / 180;

    Odom::reset_variables();

    // save odom state
    Odom::position.x_meter = xPosGlobal;
    Odom::position.y_meter = yPosGlobal;
    Odom::position.x_pct = xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    Odom::position.y_pct = yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
    Odom::position.theta = currentAbsoluteOrientation;
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
        switch (Odom::odometry_mode) {
            case OdomMode::MOTOR_IMU:
                // Odom::LPos = Odom::chassis->getLeftPosition();
                // Odom::RPos = Odom::chassis->getRightPosition();
                if (DEBUG) {
                    printf("Encoder: L=%f, R=%f\n", Odom::LPos, Odom::RPos);
                }
                break;
            case OdomMode::MOTOR_MIDTW_IMU:
                // Odom::LPos = Odom::chassis->getLeftPosition();
                // Odom::RPos = Odom::chassis->getRightPosition();
                Odom::SPos = Odom::midTW->get();
                if (DEBUG) {
                    printf("Encoder: L=%f, R=%f, S=%f\n", Odom::LPos, Odom::RPos, Odom::SPos);
                }
                break;
            case OdomMode::LEFTTW_MIDTW_IMU:
                Odom::LPos = Odom::leftTW->get();
                Odom::SPos = Odom::midTW->get();
                if (DEBUG) {
                    printf("Encoder: L=%f, S=%f\n", Odom::LPos, Odom::SPos);
                }
                break;
            case OdomMode::RIGHTTW_MIDTW_IMU:
                Odom::RPos = Odom::rightTW->get();
                Odom::SPos = Odom::midTW->get();
                if (DEBUG) {
                    printf("Encoder: R=%f, S=%f\n", Odom::RPos, Odom::SPos);
                }
                break;
        }
        

        //Calculate distance traveled by tracking each wheel (meters)
        //Converts degrees to radians
        Odom::deltaDistL = ((Odom::LPos - Odom::LPrevPos) * M_PI / 180) * Odom::WHEEL_RADIUS;
        Odom::deltaDistR = ((Odom::RPos - Odom::RPrevPos) * M_PI / 180) * Odom::WHEEL_RADIUS;
        Odom::deltaDistS = ((Odom::SPos - Odom::SPrevPos) * M_PI / 180) * Odom::WHEEL_RADIUS;

        //Update previous values to be used next loop (DEGREES)
        Odom::LPrevPos = Odom::LPos;
        Odom::RPrevPos = Odom::RPos;
        Odom::SPrevPos = Odom::SPos;

        //Total change in each of the L and R encoders since last reset (meters)
        //These are used to calculate the absolute orientation of the bot
        Odom::totalDeltaDistL += Odom::deltaDistL;
        Odom::totalDeltaDistR += Odom::deltaDistR;

        //Calculate the current absolute orientation (RADIANS)

        Odom::currentAbsoluteOrientation = Odom::THETA_START + ((Odom::imu1->get_rotation() + Odom::imu2->get_rotation()) / 2) * M_PI / 180.0;
        if (isinf(Odom::currentAbsoluteOrientation)) {
            Odom::currentAbsoluteOrientation = 0;
        }

        //Calculate the change in the angle of the bot (RADIANS)
        Odom::deltaTheta       = Odom::currentAbsoluteOrientation - Odom::previousTheta;

        //Update the previous Theta value (RADIANS)  
        Odom::previousTheta    = Odom::currentAbsoluteOrientation;

        //If we didn't turn, then we only translated
        if(Odom::deltaTheta == 0) {
            Odom::deltaXLocal  = Odom::deltaDistS;
            Odom::deltaYLocal  = Odom::deltaDistL;
        }
        //Else, caluclate the new local position
        else {
        //Calculate the changes in the X and Y values (meters)

        // The calculation assumes that the middle tracking wheel is at the front of the machine
            Odom::deltaXLocal  = 2 * sin(Odom::deltaTheta / 2.0) * ((Odom::deltaDistS / Odom::deltaTheta) - Odom::STrackRadius);
            Odom::deltaYLocal  = 2 * sin(Odom::deltaTheta / 2.0) * ((Odom::deltaDistR / Odom::deltaTheta) + Odom::RTrackRadius);
        }

        //The average angle of the robot during it's arc (RADIANS)
        Odom::avgThetaForArc   = Odom::currentAbsoluteOrientation - (Odom::deltaTheta / 2);

        Odom::deltaXGlobal     = (Odom::deltaYLocal * cos(Odom::avgThetaForArc)) - (Odom::deltaXLocal * sin(Odom::avgThetaForArc));
        Odom::deltaYGlobal     = (Odom::deltaYLocal * sin(Odom::avgThetaForArc)) + (Odom::deltaXLocal * cos(Odom::avgThetaForArc));

        //Update global positions
        Odom::xPosGlobal      += Odom::deltaXGlobal;
        Odom::yPosGlobal      += Odom::deltaYGlobal;

        Odom::position.x_meter = Odom::xPosGlobal;
        Odom::position.y_meter = Odom::yPosGlobal;
        Odom::position.x_pct   = Odom::xPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
        Odom::position.y_pct   = Odom::yPosGlobal / Constants::FIELD::FIELD_LENGTH * 100;
        Odom::position.theta   = Odom::currentAbsoluteOrientation;

        pros::delay(10);

    }
}