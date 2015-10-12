#include "BluetoothSystem.h"
#include "Robot.h"

void BluetoothSystem::init(Robot* robot)
{
    Serial.println("Initializing Bluetooth System");
    if (robot == NULL) {
        Serial.println("Error: null robot in BluetoothSystem::init");
        return;
    }
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    Serial3.begin(115200);

    // Initialize and schedule activities
    read_act.init(&robot->status, robot->addr);
    robot->schedule(read_act);

    hb_act.init(robot->addr);
    robot->schedule(hb_act);
}

void BluetoothSystem::send(fc::Message msg) {
    const size_t sz = 10;
    byte buf[sz];
    // Encode message
    int msg_len = msg.encode(buf, sz);
    // Send message
    send(buf, msg_len);
}

void BluetoothSystem::send(byte* msg, size_t len) {
    Serial3.flush();
    Serial3.write(msg, len);
}

void BluetoothSystem::ReadActivity::init(fc::MessageHandler* handler, fc::Address addr) { 
    if(handler == NULL) {
        Serial.println("Error: null handler in BluetoothSystem::ReadActivity::init");
    }
    this->handler = handler; 
    this->addr = addr; 
}

// Reads Serial3 data until there is no data to read
// or until the next full message is read, whichever
// comes first
bool BluetoothSystem::ReadActivity::run() {
    int b;
    while((b = Serial3.read()) >= 0) {
        switch(state) {
        case NO_MSG:
            if(b == 0x5f) {
                buf[0] = b;
                state = READ_LEN;
            }
            break;
        case READ_LEN:
            len = b;
            bytes_read = 0;
            if(len+1 > BUF_SIZE) {
                // Error, couldn't fit message in buffer
                state = NO_MSG;
            } else {
                // Valid length, move on
                state = READ_MSG;
            }
            break;
        case READ_MSG:
            // Read bytes until
            buf[2+bytes_read++] = b;
            if(bytes_read >= len-1) {
                fc::Message m = fc::Message::decode(buf, BUF_SIZE);
                if (m.dst() == 0 || m.dst() == addr) {
                    m.handleWith(*handler);
                }
                // Process no more than 1 message at a time
                state = NO_MSG;
                return false;
            }
            break;
        }
    }
    return false;
}

void BluetoothSystem::HeartbeatActivity::init(fc::Address addr) { this->addr = addr; }

bool BluetoothSystem::HeartbeatActivity::run() {
    // Construct heartbeat message
    fc::HeartbeatMessage hbmsg;
    hbmsg.src = addr;
    hbmsg.dst = 0;
    BluetoothSystem::send(fc::Message(hbmsg));
    // Run once a second
    waitFor(1000);
    return false;
}
