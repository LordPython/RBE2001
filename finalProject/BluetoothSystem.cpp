#include "BluetoothSystem.h"

BluetoothSystem::BluetoothSystem(fc::Address teamAddress) 
    : addr(teamAddress), time_last_heartbeat(0), _enabled(false) {}

void BluetoothSystem::init()
{
    Serial1.begin(115200);
}

void BluetoothSystem::loop() 
{
    long now = millis();
    const int sz = 10;
    byte msg[sz]; // Buffer for message
    if ((now - time_last_heartbeat) > 2*1000)
    {
        //Serial.println("Sending Heartbeat");
        fc::HeartbeatMessage hbmsg;
        hbmsg.src = addr;
        hbmsg.dst = 0;
        int msg_len = fc::Message(hbmsg).encode(msg, sz);
        master.sendPkt(msg, msg_len);
        time_last_heartbeat = now;
    }

    if(master.readPacket(msg)) {
        fc::Message m = fc::Message::decode(msg, sz);
        if (m.dst() == 0 || m.dst() == addr) {
            m.accept(*this);
        }
    }
}

fc::Availability BluetoothSystem::supply() { return _supply; }
fc::Availability BluetoothSystem::storage() { return _storage; }
bool BluetoothSystem::isEnabled() { return _enabled; }

void BluetoothSystem::handle(const fc::StorageMessage& msg) {
    //Serial.println("Got Storage Message");
    _storage = msg.availability.tubes;
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

