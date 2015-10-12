#pragma once

#include <Servo.h>
#include "Ports.h"

/**
 * A convenience class for using the Servo library
 **/
class Motor {
public:
    /**
     * Initialize the motor
     * @param port The port to attach the motor to.
     **/
    inline void init(PWMPort port) { motor.attach(port, 1000, 2000); }
    /**
     * Set the motor to a given speed
     * @param value speed to run the motor at from -90 to +90
     **/
    inline void write(int value) { motor.write(max(0,min(180,value+90))); }
private:
    Servo motor;
};
