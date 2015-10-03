#include "NavSystem.h"
#include "Robot.h"

void NavSystem::init(Robot* robot) {
    this->robot = robot;
}

void NavSystem::go(Location loc) {
    desired = loc;
}
