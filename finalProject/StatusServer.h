#pragma once

#include "Message.h"
#include <LiquidCrystal.h>

class Robot;

enum RadiationLevel {
    HIGH_RAD, LOW_RAD, NO_RAD,
};

class StatusServer : public fc::MessageHandler {
public:
    StatusServer() : lcd(40, 41, 42, 43, 44, 45) {}
    void init(Robot* robot);

    inline fc::Availability storage() { return _storage; }
    inline fc::Availability supply() { return _supply; }

    inline bool enabled() { return _enabled; }

    void setRadiationLevel(RadiationLevel lvl);

    virtual void handle(const fc::StorageMessage& msg);
    virtual void handle(const fc::SupplyMessage& msg);
    virtual void handle(const fc::StopMessage& msg);
    virtual void handle(const fc::StartMessage& msg);
private:

    void printStorage();
    void printSupply();
    void printRadiation();
    void printStopped();

    Robot* robot;

    LiquidCrystal lcd;

    fc::Availability _storage;
    fc::Availability _supply;
    RadiationLevel lvl;
    bool _enabled;
};
