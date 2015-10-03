#pragma once

#include "Arduino.h"

enum Priority {
    CONTROL_LOOP = 2,
    MAIN = 1,
};

class Activity {
public:
    /**
     * Run the task once.
     * @return whether or not to imediately requeue the task
     */
    virtual void run() = 0;
    virtual Priority priority() = 0;

    inline void stim() { waiting = false; }
    inline bool isWaiting() { if(millis() > timeout) { stim(); timeout = -1; } return waiting; }
protected:
    inline void wait() { waiting = true; timeout = -1; }
    inline void waitFor(unsigned long ms) { waiting = true; timeout = millis()+ms; }
private:
    bool waiting = false;
    unsigned long timeout = -1;
};
