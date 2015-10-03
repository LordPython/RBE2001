#pragma once

#include <Arduino.h>

enum PWMPort {
    ARM_MOTOR_PORT = 11,
    SLIDE_MOTOR_PORT = 6,
    GRIPPER_MOTOR_PORT = 7,
};

enum AnalogPort {
    ARM_POT_PORT = A11,
};

enum InterruptPort {
    SLIDE_TOP_SWITCH_PORT = 3,
    SLIDE_BOTTOM_SWITCH_PORT = 2,
};

