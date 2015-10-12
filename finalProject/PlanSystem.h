#pragma once

#include "Activity.h"
#include "NavSystem.h"

class Robot;

/**
 * System for controlling the overall execution of the robot
 **/
class PlanSystem {
public:
    /**
     * Initialize the system
     **/
    void init(Robot* robot);
    /**
     * Schedule the planning activity
     **/
    void start();
    /**
     * Wake the planning activity to proceed to
     * the next state
     **/
    void next();
private:
    //! Pointer to the robot object
    Robot* robot;

    /**
     * Activity for controlling overall execution
     **/
    class PlanActivity : public Activity {
    public:
        void init(Robot* robot);
        virtual bool run();
        virtual Priority priority() { return MAIN; }
    private:
        //! Pointer to the robot object
        Robot* robot;
        //! Possible states for overall execution
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
        Vector reactor;
    } plan_act;
};
