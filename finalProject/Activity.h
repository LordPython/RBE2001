#pragma once

#include "Arduino.h"

/**
 * Priority levels for activities.
 * CONTROL_LOOP - activities with this priority will be run once every 20ms
 * MAIN - activities with this priority will be run whenever CONTROL_LOOP activities aren't
 **/
enum Priority {
    CONTROL_LOOP = 2,
    MAIN = 1,
};

/**
 * @brief An activity is essentially an piece of code which gets run periodically
 **/
class Activity {
public:
    /**
     * @brief Run the task once.
     * @return whether or not the activity has finished
     */
    virtual bool run() = 0;
    /**
     * @brief The priority to run the activity at
     * @return Priority level for activity
     **/
    virtual Priority priority() = 0;

    /**
     * @brief Wake up the activity
     **/
    inline void stim() { waiting = false; }
    /**
     * @brief Check if the activity is currently waiting
     **/
    inline bool isWaiting() { if(millis() > timeout) { stim(); timeout = -1; } return waiting; }
protected:
    /**
     * @breif wait for an external stim()
     * To be called by activity to wait for external stim()
     * When an activity is waiting, its run() method will not be called
     **/
    inline void wait() { waiting = true; timeout = -1; }
    /**
     * @breif wait for the specified time
     * To be called by activity to wait for external stim() or
     * specified timeout
     * @param ms Timeout in ms for waiting.
     **/
    inline void waitFor(unsigned long ms) { waiting = true; timeout = millis()+ms; }
private:
    //! Keeps track of if the activity is currently waiting
    bool waiting = false;
    //! Current timeout. (timeout of -1 is max value for an unsigned long, so -1 is effectively no timeout
    unsigned long timeout = -1;
};
