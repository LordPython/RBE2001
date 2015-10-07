#pragma once

#include <QTRSensors.h>
#include "Motor.h"
#include "Activity.h"
#include "Util.h"
#include "PID.h"

class Robot;

enum Location { 
    START,
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
    NavSystem() : qtrrc8(SENSOR_PINS, NUM_SENSORS, TIMEOUT, EMITTER_PIN) {}
    void init(Robot* robot);
    void go(Location loc);
    void calibrate();

    unsigned char SENSOR_PINS[8] = {52, 53, 50, 51, 48, 49, 46, 47};
    const static int NUM_SENSORS = 8;
    const static int TIMEOUT = 2500;
    const static int EMITTER_PIN = 29;
private:

    Robot* robot;

    QTRSensorsRC qtrrc8;
    Motor left;
    Motor right;

    Location desired;
    Location current;

    enum CommandType {
        BACK_UP, FORWARD, TURN_AROUND, TURN_LEFT, TURN_RIGHT, FOLLOW_LIMIT, FOLLOW_COUNT, DONE,
    };
    
    struct Command {
        CommandType type;
        int intersections;

        inline Command() : type(DONE), intersections(1) {}
        inline Command(CommandType type) : type(type), intersections(1) {}
        inline Command(CommandType type, int intersections) : type(type), intersections(intersections) {}
    };

    Command current_command;

    CircularQueue<Command, 10> commands;

    void drive(int left, int right);
    void stop() { drive(0,0); }
    void next();

    friend class NavActivity;
    class NavActivity : public Activity {
    public:
        void init(NavSystem* nav);
        void resetStateTime();
        virtual void run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        int intersections;
        bool done_pause;
        NavSystem* nav;
        PID pid;
        unsigned long lastStateTime;
        unsigned long lastLineTime;
        bool followLine();
    } nav_act;
};
