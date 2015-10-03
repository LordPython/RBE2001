#include "Robot.h"
#include "ArmSystem.h"
#include "Ports.h"

const int k_arm_up_setpoint = 297;
const int k_arm_down_setpoint = 670;

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

void ArmSystem::ArmIDA::init(Robot* robot) {
    this->robot = robot;
}

void ArmSystem::setArm(Setpoint set) { arm_ida.setArm(set); }
void ArmSystem::setSlide(Setpoint set) { arm_ida.setSlide(set); }
void ArmSystem::setGripper(Setpoint set) { arm_ida.setGripper(set); }

void ArmSystem::ArmActivity::init(ArmIDA* arm_ida) {
    this->arm_ida = arm_ida;
    arm_ida->setArm(DOWN);
    pid.init(0.8, 0.01, 1.0, 5);
    arm_motor.init(ARM_MOTOR_PORT);
}

void ArmSystem::ArmActivity::run() {
    Setpoint desired = arm_ida->getArm();
    int setpoint = desired == UP ? k_arm_up_setpoint : k_arm_down_setpoint;
    int current  = analogRead(ARM_POT_PORT);
    int error = current - setpoint;
    //if (
    arm_motor.write(pid.calc(error));
}

void ArmSystem::SlideActivity::init(ArmIDA* arm_ida) {
    this->arm_ida = arm_ida;
    arm_ida->setSlide(DOWN);
    slide_motor.init(SLIDE_MOTOR_PORT);
}

void ArmSystem::SlideActivity::run() {
    if (arm_ida->getSlide() == DOWN && digitalRead(SLIDE_BOTTOM_SWITCH_PORT)) {
        slide_motor.write(40);
    } else if (arm_ida->getSlide() == UP && digitalRead(SLIDE_TOP_SWITCH_PORT)) {
        slide_motor.write(-40);
    } else {
        slide_motor.write(0);
    }
}

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
