#pragma once

#include <stdint.h>
#include <stddef.h>

#include "Activity.h"

// A simple list
template<typename T, size_t S>
class List {
public:
    List() : _size(0) {}

    inline size_t size() {
        return _size;
    }

    void insert(T t) {
        if(_size < S) {
            array[_size++] = t;
        } else {
#ifdef DEBUG
            Serial.println("Array overflow");
            Serial.flush();
#endif
        }
    }

    bool remove(T t) {
        for(int i = 0; i < _size; ++i) {
            if(array[i] == t) {
                --_size;
                for (int j = i; j < _size; ++j) {
                    array[j] = array[j+1];
                }
                return true;
            }
        }
        return false;
    }

    T operator[](int i) {
#ifdef DEBUG
        if (i >= _size) {
            Serial.println("Out of bounds array access");
            Serial.flush();
        }
#endif
        return array[i];
    }
private:
    size_t _size;
    T array[S];
};

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
