#include "Robot.h"

void Robot::init(fc::Address addr) {
    Scheduler::init();
    this->addr = addr;
    // Initialze the subsystems
    bluetooth.init(this);
    //status.init(this);
    planner.init(this);
    //nav.init(this);
    arm.init(this);
}
