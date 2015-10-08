#include "Robot.h"
#include "ArmSystem.h"
#include "Ports.h"

// analog setpoints for arm up and down positions
const int k_arm_up_setpoint = 280;
const int k_arm_down_setpoint = 676;

void ArmSystem::init(Robot* robot) {
    pinMode(SLIDE_TOP_SWITCH_PORT, INPUT_PULLUP);
    pinMode(SLIDE_BOTTOM_SWITCH_PORT, INPUT_PULLUP);

    arm_ida.init(robot);

    arm_act.init(&arm_ida);
    slide_act.init(&arm_ida);
    gripper_act.init(&arm_ida);

    robot->schedule(arm_act);
    robot->schedule(slide_act);
    robot->schedule(gripper_act);
}

void ArmSystem::setArm(Setpoint set) { arm_ida.setArm(set); }
void ArmSystem::setSlide(Setpoint set) { arm_ida.setSlide(set); }
void ArmSystem::setGripper(Setpoint set) { arm_ida.setGripper(set); }

// -------------------------------------------------------- //
//                        Arm IDA                           //
// -------------------------------------------------------- //

void ArmSystem::ArmIDA::init(Robot* robot) {
    this->robot = robot;
}

void ArmSystem::ArmIDA::armAt(Setpoint set) {
    if(arm_cur != set && arm_desired == set) {
        arm_cur = set;
        if (slide_cur == slide_desired) {
            done();
        }
    }
}

void ArmSystem::ArmIDA::slideAt(Setpoint set) {
    
    if(slide_cur != set && slide_desired == set) {
        slide_cur = set;
        if (arm_cur == arm_desired) {
            done();
        }
    }
}

void ArmSystem::ArmIDA::done() {
    // Done with current action, let the planner
    // know it's time to continue.
    robot->planner.next();
}

// -------------------------------------------------------- //
//                     Arm Activity                        //
// -------------------------------------------------------- //

void ArmSystem::ArmActivity::init(ArmIDA* arm_ida) {
    this->arm_ida = arm_ida;
    arm_ida->setArm(DOWN);
    pid.init(0.8, 0.005, 1.0, 5);
    arm_motor.init(ARM_MOTOR_PORT);
}

void ArmSystem::ArmActivity::run() {
    Setpoint desired = arm_ida->getArm();
    int setpoint = desired == UP ? k_arm_up_setpoint : k_arm_down_setpoint;
    int current  = analogRead(ARM_POT_PORT);
    int error = current - setpoint;
    if (abs(error) < 5) {
        arm_ida->armAt(desired);
    }
    arm_motor.write(pid.calc(error));
}

// -------------------------------------------------------- //
//                      Slide Activity                      //
// -------------------------------------------------------- //

void ArmSystem::SlideActivity::init(ArmIDA* arm_ida) {
    this->arm_ida = arm_ida;
    arm_ida->setSlide(DOWN);
    slide_motor.init(SLIDE_MOTOR_PORT);
}

void ArmSystem::SlideActivity::run() {
    Setpoint desired = arm_ida->getSlide();
    // the switches are normally open, so a digital 1 means
    // not pressed.
    if (desired == DOWN && digitalRead(SLIDE_BOTTOM_SWITCH_PORT)) {
        slide_motor.write(40);
    } else if (desired == UP && digitalRead(SLIDE_TOP_SWITCH_PORT)) {
        slide_motor.write(-40);
    } else {
        arm_ida->slideAt(desired);
        slide_motor.write(0);
    }
}

// -------------------------------------------------------- //
//                    Gripper Activity                      //
// -------------------------------------------------------- //

void ArmSystem::GripperActivity::init(ArmIDA* arm_ida) {
    this->arm_ida = arm_ida;
    arm_ida->setGripper(OPEN);
    gripper_motor.init(GRIPPER_MOTOR_PORT);
}

void ArmSystem::GripperActivity::run() {
    if (arm_ida->getGripper() == OPEN) {
        gripper_motor.write(0);
    } else {
        gripper_motor.write(90);
    }
}
