#include "main.h"

/**
 * @brief Construct a new Catapult:: Catapult object
 * 
 * @param core Core object pointer
 */
Catapult::Catapult(struct Core* core) {
    this->triggered = false;
    this->voltage = 8000;
    this->core = core;
    this->reposition();
    this->core->catapult_motor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    this->shooting_task = std::move(std::make_unique<pros::Task>(this->shooting_loop_trampoline, this, "shooting loop"));
}
/**
 * @brief Destroy the Catapult:: Catapult object
 * 
 */
Catapult::~Catapult() {
    shooting_task->remove();
    shooting_task.reset(nullptr);
}


/**
 * @brief Reload the catapult to a charged position
 * 
 */
void Catapult::reposition() {
    // rotate until it is loaded again
    while (this->core->catapult_load_sensor->get_value() == 0) {
        this->core->catapult_motor->moveVoltage(this->voltage);
        pros::delay(20);
    }
    this->core->catapult_motor->moveVoltage(0);
}

/**
 * @brief Turn on/off the piston booster
 * 
 * @param use_boost whether piston booster should turn on
 */
void Catapult::set_boost(bool use_boost) {
    this->core->piston_booster_left->set_value(use_boost);
    this->core->piston_booster_right->set_value(use_boost);
}

/**
 * @brief Request to shoot
 * 
 */
void Catapult::fire() {
    this->triggered = true;
}

void Catapult::wait_until_reloaded() {
    while (triggered) {
        ;
    }
}

/**
 * @brief Trampoline function for shooting background task
 * 
 * @param iparam instance of the Catapult class
 */
void Catapult::shooting_loop_trampoline(void* iparam) {
    if(iparam){
        Catapult* that = static_cast<Catapult*>(iparam);
        that->shooting_loop();
        pros::delay(10);
    }
}

/**
 * @brief Background task for handling shooting requests
 * 
 */
void Catapult::shooting_loop() {
    while (true) {
        if (this->triggered) {
            // rotate until it is not loaded
            while (this->core->catapult_load_sensor->get_value() != 0) {
                this->core->catapult_motor->moveVoltage(this->voltage);
                pros::delay(20);
            }
            this->reposition();
            this->triggered = false;
        }
        pros::delay(20);
    }
}