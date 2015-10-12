#pragma once

#include <Arduino.h>

/**
 * Motor ports
 **/
enum PWMPort {
    ARM_MOTOR_PORT = 11,
    SLIDE_MOTOR_PORT = 6,
    GRIPPER_MOTOR_PORT = 7,
    LEFT_DRIVE_PORT = 4,
    RIGHT_DRIVE_PORT = 5,
};

/**
 * Analog ports
 **/
enum AnalogPort {
    ARM_POT_PORT = A11,
};

/**
 * Interrupt ports
 **/
enum InterruptPort {
    SLIDE_TOP_SWITCH_PORT = 3,
    SLIDE_BOTTOM_SWITCH_PORT = 2,
    FRONT_BUMP_PORT = 18,
    START_BUTTON_PORT = 19,
};

