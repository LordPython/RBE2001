#pragma once

#include "Scheduler.h"
#include "PlanSystem.h"
#include "BluetoothSystem.h"
#include "StatusServer.h"
#include "ArmSystem.h"
#include "NavSystem.h"

struct Robot : public Scheduler {
    void init(fc::Address addr);

    fc::Address addr;

    BluetoothSystem bluetooth;
    ArmSystem arm;
    PlanSystem planner;
    NavSystem nav;
    StatusServer status;
};
