#pragma once

#include <stdint.h>
#include <stddef.h>

#include "Activity.h"
#include "Util.h"

// A very simple cooperative multitasking scheduler
class Scheduler {
public:
    void init();

    // Schedule a task to run (by putting it in the appropriate queue)
    void schedule(Activity& act);
    // Remove an Activity from the queue
    void deschedule(Activity& act);

    // Run the next task
    void runNext();
private:
    List<Activity*, 10> control_loop_queue;
    List<Activity*, 10> main_queue;
    int curActIdx;
};
