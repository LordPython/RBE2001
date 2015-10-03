#include "Scheduler.h"
#include "Robot.h"
#include "TimerOne.h"

static volatile bool run_control_loops;

// 50 ms period for control loops
const unsigned long k_controlLoopPeriod = 50000;

void queueControlLoops() {
    run_control_loops = true;
}

void Scheduler::init() {
    Serial.println("init scheduler");
    Serial.flush();
    Timer1.initialize();
    Timer1.attachInterrupt(queueControlLoops, k_controlLoopPeriod);
    curActIdx = 0;
}

void Scheduler::schedule(Activity& act) {
    switch(act.priority()) {
    case CONTROL_LOOP:
        Serial.println("Scheduling a control loop");
        Serial.flush();
        control_loop_queue.insert(&act);
        break;
    case MAIN:
    default:
        Serial.println("Scheduling a main loop");
        Serial.flush();
        main_queue.insert(&act);
        break;
    }
}

void Scheduler::deschedule(Activity& act) {
    switch(act.priority()) {
    case CONTROL_LOOP:
        control_loop_queue.remove(&act);
        break;
    case MAIN:
    default:
        main_queue.remove(&act);
        break;
    }
}

void Scheduler::runNext() {
    // Disable interrupts
    uint8_t saveReg = SREG;
    cli();
    // check for control loop flag
    if(run_control_loops) {
        // reset control loop flag
        run_control_loops = false;
        // restore interrupts
        SREG = saveReg;
        // run all control loops
        for(int i = 0; i < control_loop_queue.size(); ++i) {
            if(!control_loop_queue[i]->isWaiting()) {
                control_loop_queue[i]->run();
            }
        }
    } else {
        // restore interrupts
        SREG = saveReg;
        if(curActIdx < main_queue.size() && main_queue.size() > 0) {
            // run the next activity
            if(!main_queue[curActIdx]->isWaiting()) {
                main_queue[curActIdx]->run();
            }
            ++curActIdx;
        } else {
            curActIdx = 0;
        }
    }
}
