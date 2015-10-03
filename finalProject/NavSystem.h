#pragma once

#include "Motor.h"

class Robot;

enum Location { 
    REACTOR_A,
    REACTOR_B,
    STORAGE_1,
    STORAGE_2,
    STORAGE_3,
    STORAGE_4,
    SUPPLY_1,
    SUPPLY_2,
    SUPPLY_3,
    SUPPLY_4,
};

class NavSystem {
public:
    void init(Robot* robot);
    void go(Location loc);
private:
    Robot* robot;

    Location desired;
};
