#pragma once

#include <QTRSensors.h>
#include "Motor.h"
#include "Activity.h"
#include "Util.h"
#include "PID.h"

class Robot;

class NavSystem {
public:
    NavSystem() : qtrrc8(SENSOR_PINS, NUM_SENSORS, TIMEOUT, EMITTER_PIN) {}
    void init(Robot* robot);
    void start();
    void go(Vector new_pos);
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

    Vector desired_pos;
    Vector desired_dir;
    Vector current_pos;
    Vector current_dir;

    enum CommandType {
        BACK_UP, FORWARD, TURN_AROUND, TURN_LEFT, TURN_RIGHT, FOLLOW_LIMIT, FOLLOW_COUNT, DONE,
    };
    
    struct Command {
        CommandType type;
        int data;

        inline Command() : type(DONE), data(0) {}
        inline Command(CommandType type) : type(type), data(0) {}
        inline Command(CommandType type, int intersections) : type(type), data(intersections) {}
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
        virtual bool run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        NavSystem* nav;
        PID pid;
        unsigned long lastStateTime;
        unsigned long lastLineTime;
        void followLine(long position);
    } nav_act;
};
