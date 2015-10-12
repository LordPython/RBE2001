#include "PlanSystem.h"
#include "Robot.h"

void PlanSystem::init(Robot* robot) {
    this->robot = robot;
    Serial.println("Plan init");
    plan_act.init(robot);
}

void PlanSystem::start() {
    robot->schedule(plan_act);
}

void PlanSystem::next() {
    plan_act.stim();
}

void PlanSystem::PlanActivity::init(Robot* robot) {
    this->robot = robot;
    reactor = REACTOR_A;
    // Wait for robot to get into default state.
    wait();
}

bool PlanSystem::PlanActivity::run() {
    Serial.println("Running planner");
    switch(state) {
    case GO_TO_REACTOR_1:
        Serial.println("Go to reactor 1");
        robot->nav.go(reactor);
        wait();
        state = GRIP_REACTOR_1;
        break;
    case GRIP_REACTOR_1: {
        Serial.println("Grip reactor 1");
        robot->arm.setGripper(CLOSE);
        Availability storage = robot->status.storage();
        Availability supply = robot->status.supply();
        Serial.print("Storage status: ");
        Serial.print(storage.tube1);
        Serial.print(storage.tube2);
        Serial.print(storage.tube3);
        Serial.println(storage.tube4);
        Serial.print("Supply status: ");
        Serial.print(supply.tube1);
        Serial.print(supply.tube2);
        Serial.print(supply.tube3);
        Serial.println(supply.tube4);
        waitFor(1000);
        state = SLIDE_REACTOR_1;//GRIP_REACTOR_1;//SLIDE_REACTOR_1;
        break;
    }
    case SLIDE_REACTOR_1:
        Serial.println("Slide reactor 1");
        robot->arm.setSlide(UP);
        robot->status.setRadiationLevel(LOW_RAD);
        wait();
        state = ARM_REACTOR_1;
        break;
    case ARM_REACTOR_1:
        Serial.println("Arm reactor 1");
        robot->arm.setArm(UP);
        wait();
        state = GO_TO_STORAGE;
        break;
    case GO_TO_STORAGE: {
        Serial.println("Go to storage");
        Availability storage = robot->status.storage();
        Serial.print("Storage status: ");
        Serial.print(storage.tube1);
        Serial.print(storage.tube2);
        Serial.print(storage.tube3);
        Serial.println(storage.tube4);
        if(!storage.tube1) robot->nav.go(STORAGE_1);
        else if(!storage.tube2) robot->nav.go(STORAGE_2);
        else if(!storage.tube3) robot->nav.go(STORAGE_3);
        else if(!storage.tube4) robot->nav.go(STORAGE_4);
        // Pick empty storage
        // Nav there
        wait();
        state = SLIDE_STORAGE;
        break;
    }
    case SLIDE_STORAGE:
        robot->arm.setSlide(DOWN);
        wait();
        state = GRIP_STORAGE;
        break;
    case GRIP_STORAGE:
        robot->arm.setGripper(OPEN);
        // No feedback on the gripper, wait 1 second
        waitFor(1000);
        state = GO_TO_SUPPLY;
        robot->status.setRadiationLevel(NO_RAD);
        break;
    case GO_TO_SUPPLY: {      
        Availability supply = robot->status.supply();
        if(supply.tube1) robot->nav.go(SUPPLY_1);
        else if(supply.tube2) robot->nav.go(SUPPLY_2);
        else if(supply.tube3) robot->nav.go(SUPPLY_3);
        else if(supply.tube4) robot->nav.go(SUPPLY_4);
        else {
            // Problem not solvable. Just go to supply 1
            robot->nav.go(SUPPLY_1);
        }
        // pick filled supply
        // Nav there
        wait();
        state = GRIP_SUPPLY;
        break;
    }
    case GRIP_SUPPLY:
        robot->arm.setGripper(CLOSE);
        waitFor(1000);
        state = SLIDE_SUPPLY;
        break;
    case SLIDE_SUPPLY:
        robot->arm.setSlide(UP);
        robot->status.setRadiationLevel(HIGH_RAD);
        wait();
        state = GO_TO_REACTOR_2;
        break;
    case GO_TO_REACTOR_2:
        robot->nav.go(reactor);
        wait();
        state = ARM_REACTOR_2;
        break;
    case ARM_REACTOR_2:
        robot->arm.setArm(DOWN);
        wait();
        state = SLIDE_REACTOR_2;
        break;
    case SLIDE_REACTOR_2:
        robot->arm.setSlide(DOWN);
        wait();
        state = GRIP_REACTOR_2;
        break;
    case GRIP_REACTOR_2:
        robot->arm.setGripper(OPEN);
        waitFor(1000);
        if(reactor == REACTOR_A) {
            reactor = REACTOR_B;
            state = GO_TO_REACTOR_1;
        } else {
            state = END;
        }
        robot->status.setRadiationLevel(NO_RAD);
        break;
    case END:
        return true;
    default:
        break;
    }
    return false;
}
