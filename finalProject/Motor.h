#pragma once

#include <Servo.h>
#include "Ports.h"

class Motor {
public:
    inline void init(PWMPort port) { motor.attach(port, 1000, 2000); }
    inline void write(int value) { motor.write(max(0,min(180,value+90))); }
private:
    Servo motor;
};
