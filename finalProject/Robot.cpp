#include "Robot.h"

void Robot::init(Address addr) {
    // Initialize scheduler
    Scheduler::init();

    this->addr = addr;
    // Initialze the subsystems
    bluetooth.init(this);
    status.init(this);
    planner.init(this);
    nav.init(this);
    arm.init(this);
    started = false;
}

void Robot::start() {
    if(!started) {
        Serial.println("Starting!");
        started = true;
        planner.start();
        nav.start();
        arm.start();
    }
}

