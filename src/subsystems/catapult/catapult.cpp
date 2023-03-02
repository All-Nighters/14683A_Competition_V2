#include "main.h"

bool Catapult::continue_shooting = true;


/**
 * @brief Construct a new Catapult:: Catapult object
 * 
 * @param core Core object pointer
 */
Catapult::Catapult(struct Core* core) {
    this->triggered = false;
    this->voltage = -12000;
    this->fire_delay = 0;
    this->core = core;
    this->use_boost = false;

    // this->rotation_sensor = std::make_unique<pros::Task>();
    this->core->catapult_motor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    this->shooting_task = std::move(std::make_unique<pros::Task>(this->shooting_loop_trampoline, this, "shooting loop"));
    Catapult::continue_shooting = true;
    printf("finished cata %d\n", Catapult::continue_shooting);
}
/**
 * @brief Destroy the Catapult:: Catapult object
 * 
 */
Catapult::~Catapult() {
    Catapult::continue_shooting = false;
    this->shooting_task->suspend();
    this->shooting_task->remove();
    this->shooting_task.reset(nullptr);
    this->core->catapult_motor->moveVoltage(0);
    this->core->piston_booster->set_value(false);
    printf("Catapult destroyed %d\n", continue_shooting);
}

void Catapult::set_voltage(int voltage) {
    this->voltage = -voltage;
}

/**
 * @brief Reload the catapult to a charged position
 * 
 */
void Catapult::reposition() {
    // rotate until it is loaded again
    while (this->core->catapult_load_sensor->get_value() == 0) {
        this->core->piston_booster->set_value(false);
        this->core->catapult_motor->moveVoltage(this->voltage);
        pros::delay(10);
    }

    // this->rotation_sensor.reset_position();
    // while (this->rotation_sensor->get_rotation() < 500) { // 100 centedegrees is 1 degree
    //     this->core->catapult_motor->moveVoltage(this->voltage);
    //     pros::delay(10);
    // }

    this->core->piston_booster->set_value(this->use_boost);
    this->core->catapult_motor->moveVoltage(0);
}

/**
 * @brief Turn on/off the piston booster
 * 
 * @param use_boost whether piston booster should turn on
 */
void Catapult::set_boost(bool use_boost) {
    this->use_boost = use_boost;
}

/**
 * @brief Request to shoot.
 * 
 * To shoot synchronously, add wait_until_reloaded() right after
 * calling fire().
 * 
 * @param fire_delay Miliseconds delayed between call and shot
 */
void Catapult::fire(int fire_delay) {
    this->fire_delay = fire_delay;
    this->triggered = true;
}

/**
 * @brief Wait until the catapult is reloaded
 * 
 */
void Catapult::wait_until_reloaded() {
    while (this->triggered) {
        pros::delay(10);
    }
}

/**
 * @brief Is the catapult reloaded
 * 
 */
bool Catapult::is_reloaded(){
    return !this->triggered;
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
    Catapult::continue_shooting = true;
    while (Catapult::continue_shooting) {
        this->core->piston_booster->set_value(this->use_boost);
        if (this->triggered) {
            if (this->fire_delay > 0) {
                pros::delay(this->fire_delay);
            }
            this->fire_delay = 0;
            // rotate until it is not loaded
            while (this->core->catapult_load_sensor->get_value() != 0) {
                this->core->catapult_motor->moveVoltage(this->voltage);
                pros::delay(20);
            }
            this->core->catapult_motor->moveVoltage(0);
            pros::delay(200);
            this->reposition();
            this->triggered = false;
        }
        this->reposition();
        pros::delay(20);
    }
}

void Catapult::reset() {
    this->shooting_task->remove();
    this->shooting_task.reset(nullptr);
    this->core->catapult_motor->moveVoltage(0);
    this->shooting_task = std::move(std::make_unique<pros::Task>(this->shooting_loop_trampoline, this, "shooting loop"));
}