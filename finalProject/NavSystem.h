#pragma once

#include <QTRSensors.h>
#include "Motor.h"
#include "Activity.h"
#include "Util.h"
#include "PID.h"

class Robot;

class NavSystem {
public:
    /**
     *  Construct the system
     */
    NavSystem() : qtrrc8(SENSOR_PINS, NUM_SENSORS, TIMEOUT, EMITTER_PIN) {}
    /**
     *  Initialize the nav system
     */
    void init(Robot* robot);
    /**
     *  Schedules the navigation activity
     */
    void start();
    /**
     *  Path plan to new position, then set up the
     *  system to go there.
     *  @param new_pos Position to navigate to
     */
    void go(Vector new_pos);
    /**
     *  Called to calibrate the line sensor. This should be called
     *  repeatedly while exposing the sensor the the range of values
     *  for the line.
     */
    void calibrate();

    //! Number of line sensors
    const static int NUM_SENSORS = 8;
    //! Pins for line sensor
    unsigned char SENSOR_PINS[NUM_SENSORS] = {52, 53, 50, 51, 48, 49, 46, 47};
    //! Timeout (in us) for reading the line sensors
    const static int TIMEOUT = 2500;
    //! Pin to control the emmiters for the line sensor
    const static int EMITTER_PIN = 29;
private:

    //! Pointer to robot object
    Robot* robot;
    //! Line sensor
    QTRSensorsRC qtrrc8;
    //! Right drive Motor
    Motor left;
    //! Left drive Motor
    Motor right;
    //! Desired position
    Vector desired_pos;
    //! Desired facing direciton
    Vector desired_dir;
    //! Current position
    Vector current_pos;
    //! Current facing direction
    Vector current_dir;

    /**
     * The possible individual steps used to perform navigation
     **/
    enum CommandType {
        BACK_UP, FORWARD, TURN_AROUND, TURN_LEFT, TURN_RIGHT, FOLLOW_LIMIT, FOLLOW_COUNT, DONE,
    };

    /**
     * Wrapper for command type since the FOLLOW_COUNT command
     * needs a count to keep track of how man intersecitons have been seen,
     * and some other commands use the data to keep track of state of the command
     **/
    struct Command {
        CommandType type;
        int data;

        inline Command() : type(DONE), data(0) {}
        inline Command(CommandType type) : type(type), data(0) {}
        inline Command(CommandType type, int intersections) : type(type), data(intersections) {}
    };

    //! Current command for navigation
    Command current_command;

    //! Queue of commands to run
    CircularQueue<Command, 10> commands;

    /**
     * Run the drive motors at the given speeds (-90 to +90)
     * @param left speed to run the left motor at
     * @param right speed to run the right motor at
     **/
    void drive(int left, int right);
    /**
     * Stop both motors
     **/
    inline void stop() { drive(0,0); }
    /**
     *  Wakes up planning activity
     **/
    void next();

    /**
     * Activity that takes care of performing navigation
     **/
    friend class NavActivity;
    class NavActivity : public Activity {
    public:
        void init(NavSystem* nav);
        void resetStateTime();
        virtual bool run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        //! Pointer to nav system
        NavSystem* nav;
        //! Pid for following the line
        PID pid;
        //! Time of the last state/command transitition
        unsigned long lastStateTime;
        //! Time the last intersection was seen
        unsigned long lastLineTime;
        /**
         * Drives the motors to follow the line
         * @param position current position of the line
         **/
        void followLine(long position);
    } nav_act;
};
