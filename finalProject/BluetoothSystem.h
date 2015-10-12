#pragma once

#include <Arduino.h>
#include "Message.h"
#include "Activity.h"

// Can't include robot.h (circular dependency)
// so just declare robot here.
class Robot;

/**
 * System for bluetooth communication
 **/
class BluetoothSystem : private MessageHandler {
public:
    /**
     * Send a message to the field control computer.
     * @param msg Message to send
     **/
    static void send(Message msg);

    /**
     * Initialze the system
     * @param robot Pointer to robot object
     **/
    void init(Robot* robot);
private:
    //! Pointer to robot object.
    Robot* robot;

    /**
     * Send a series of bytes over bluetooth
     * @param msg Buffer of bytes to send.
     * @param len Number of bytes to send.
     **/
    static void send(byte* msg, size_t len);

    /**
     * Activity to read incoming messages
     **/
    class ReadActivity : public Activity {
    public:
        void init(MessageHandler* handler, Address addr);
        virtual bool run();
        virtual Priority priority() { return MAIN; }
    private:
        /**
         * Possible states of reading messages
         * (This activity is implemented as a state machine)
         **/
        enum {
            NO_MSG, READ_LEN, READ_MSG,
        } state;

        //! Size of buffer used for reading messages
        static const size_t BUF_SIZE = 10;
        //! Buffer used to read in messages
        byte buf[BUF_SIZE];

        //! Read length byte
        int len;
        //! Bytes read so far
        int bytes_read;

        //! Handler for when a message has been recieved
        MessageHandler* handler;
        //! Address of this robot
        Address addr;
    } read_act;

    /**
     * Activity to send a heartbeat message once a second
     **/
    class HeartbeatActivity : public Activity {
    public:
        void init(Address addr);
        virtual bool run();
        virtual Priority priority() { return MAIN; }
    private:
        //! Address of this robot
        Address addr;
    } hb_act;
};
