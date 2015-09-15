#pragma once

#include <BluetoothMaster.h>
#include "Message.h"

class BluetoothSystem : private fc::MessageHandler {
public:
    BluetoothSystem(fc::Address robotAddress);

    fc::Availability storage();
    fc::Availability supply();
    bool isEnabled();

    void init();
    void loop();
private:
    long time_last_heartbeat;
    fc::Address addr;
    fc::Availability _storage, _supply;
    bool _enabled;
    BluetoothMaster master;

    virtual void handle(const fc::StorageMessage& msg);
    virtual void handle(const fc::SupplyMessage& msg);
    virtual void handle(const fc::StopMessage& msg);
    virtual void handle(const fc::StartMessage& msg);
};
