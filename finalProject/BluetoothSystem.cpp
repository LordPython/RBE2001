#include "BluetoothSystem.h"

BluetoothSystem::BluetoothSystem(fc::Address teamAddress) 
    : addr(teamAddress), time_last_heartbeat(0), 
      _enabled(false), serial(Serial3) {}

void BluetoothSystem::init()
{
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    serial.begin(115200);
}

void BluetoothSystem::loop() 
{
    long now = millis();
    const int sz = 10;
    byte msg[sz]; // Buffer for message
    if ((now - time_last_heartbeat) > 2*1000)
    {
        fc::HeartbeatMessage hbmsg;
        hbmsg.src = addr;
        hbmsg.dst = 0;
        int msg_len = fc::Message(hbmsg).encode(msg, sz);
        send(msg, msg_len);
        time_last_heartbeat = now;
    }

    if(read(msg, sz)) {
        fc::Message m = fc::Message::decode(msg, sz);
        if (m.dst() == 0 || m.dst() == addr) {
            m.handleWith(*this);
        }
    }
}

void BluetoothSystem::send(byte* msg, size_t len) {
    serial.flush();
    serial.write(msg, len);
}

bool BluetoothSystem::read(byte* buf, size_t sz) {
    if (sz <= 6) return false;
    int len;
    byte b;
    unsigned char timeout;
    while(serial.available()) {
        b = serial.read();
        if(b == 0x5f) {
            buf[0] = b;
            timeout = 255;
            while(serial.available() == 0) {
                if(--timeout == 0) return false;
                delay(1);
            }
            len = serial.read();
            if (len+1 > sz) return false;
            buf[1] = len;
            timeout = 255;
            while(serial.available() < len - 1) {
                if(--timeout == 0) return false;
                delay(1);
            }
            for (int i = 0; i < len - 1; ++i) {
                buf[2+i] = serial.read();
            }
            return true;
        }
    }
    return false;
}

fc::Availability BluetoothSystem::supply() { return _supply; }
fc::Availability BluetoothSystem::storage() { return _storage; }
bool BluetoothSystem::isEnabled() { return _enabled; }

void BluetoothSystem::handle(const fc::StorageMessage& msg) {
    _storage = msg.availability.tubes;
    Serial.println(msg.availability.byte);
}

void BluetoothSystem::handle(const fc::SupplyMessage& msg) {
    _supply = msg.availability.tubes;
}

void BluetoothSystem::handle(const fc::StopMessage& msg) {
    _enabled = false;
}

void BluetoothSystem::handle(const fc::StartMessage& msg) {
    _enabled = true;
}

