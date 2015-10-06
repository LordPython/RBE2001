#include "PlanSystem.h"
#include "Robot.h"

void PlanSystem::init(Robot* robot) {
    Serial.println("Plan init");
    plan_act.init(robot);
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

void PlanSystem::PlanActivity::run() {
    Serial.println("Running planner");
    switch(state) {
    case GO_TO_REACTOR_1:
        Serial.println("Go to reactor 1");
        robot->nav.go(reactor);
        wait();
        state = GRIP_REACTOR_1;
        break;
    case GRIP_REACTOR_1:
        Serial.println("Grip reactor 1");
        robot->arm.setGripper(CLOSE);
        waitFor(1000);
        state = SLIDE_REACTOR_1;
        break;
    case SLIDE_REACTOR_1:
        Serial.println("Slide reactor 1");
        robot->arm.setSlide(UP);
        robot->status.setRadiationLevel(fc::RadiationMessage::SPENT);
        wait();
        state = ARM_REACTOR_1;
        break;
    case ARM_REACTOR_1:
        Serial.println("Arm reactor 1");
        robot->arm.setArm(UP);
        wait();
        state = GO_TO_STORAGE;
        break;
    case GO_TO_STORAGE:
        Serial.println("Go to storage");
        robot->nav.go(STORAGE_1);
        // Pick empty storage
        // Nav there
        wait();
        state = SLIDE_STORAGE;
        break;
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
        break;
    case GO_TO_SUPPLY:
        robot->nav.go(SUPPLY_3);
        // pick filled supply
        // Nav there
        wait();
        state = GRIP_SUPPLY;
        break;
    case GRIP_SUPPLY:
        robot->arm.setGripper(CLOSE);
        waitFor(1000);
        state = SLIDE_SUPPLY;
        break;
    case SLIDE_SUPPLY:
        robot->arm.setSlide(UP);
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
        break;
    case END:
    default:
        break;
    }
}
