#pragma once

#include "Scheduler.h"
#include "PlanSystem.h"
#include "BluetoothSystem.h"
#include "StatusServer.h"
#include "ArmSystem.h"
#include "NavSystem.h"

struct Robot : public Scheduler {
    /**
     * Initialize the robot and all its subsystems
     */
    void init(Address addr);
    /** 
     * Starts the activities necessary to run the robot
     */
    void start();

    Address addr;

    BluetoothSystem bluetooth;
    ArmSystem arm;
    PlanSystem planner;
    NavSystem nav;
    StatusServer status;
private:
    bool started;
};
