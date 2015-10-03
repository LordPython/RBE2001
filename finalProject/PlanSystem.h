#pragma once

#include "Activity.h"
#include "NavSystem.h"

class Robot;

class PlanSystem {
public:
    void init(Robot* robot);
    void next();
private:
    class PlanActivity : public Activity {
    public:
        void init(Robot* robot);
        virtual void run();
        virtual Priority priority() { return MAIN; }
    private:
        Robot* robot;
        enum {
            GO_TO_REACTOR_1,
            GRIP_REACTOR_1,
            SLIDE_REACTOR_1,
            ARM_REACTOR_1,
            GO_TO_STORAGE,
            SLIDE_STORAGE,
            GRIP_STORAGE,
            GO_TO_SUPPLY,
            GRIP_SUPPLY,
            SLIDE_SUPPLY,
            GO_TO_REACTOR_2,
            ARM_REACTOR_2,
            SLIDE_REACTOR_2,
            GRIP_REACTOR_2,
            END,
        } state;
        Location reactor;
    } plan_act;
};
