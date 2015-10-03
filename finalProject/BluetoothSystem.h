#pragma once

#include <Arduino.h>
#include "Message.h"
#include "Activity.h"

class Robot;

class BluetoothSystem : private fc::MessageHandler {
public:
    fc::Availability storage();
    fc::Availability supply();

    static void send(fc::Message msg);

    void init(Robot* robot);
private:
    Robot* robot;

    static void send(byte* msg, size_t len);

    class ReadActivity : public Activity {
    public:
        void init(MessageHandler* handler, fc::Address addr);
        virtual void run();
        virtual Priority priority() { return MAIN; }
    private:
        enum {
            NO_MSG, READ_LEN, READ_MSG,
        } state;

        static const size_t BUF_SIZE = 10;
        byte buf[BUF_SIZE];

        int len;
        int bytes_read;

        MessageHandler* handler;
        fc::Address addr;
    } read_act;

    class HeartbeatActivity : public Activity {
    public:
        void init(fc::Address addr);
        virtual void run();
        virtual Priority priority() { return MAIN; }
    private:
        fc::Address addr;
    } hb_act;
};
