#pragma once

#include "Message.h"

class Robot;

class StatusServer : public fc::MessageHandler {
public:
    void init(Robot* robot);

    fc::Availability storage();
    fc::Availability supply();

    inline bool enabled() { return _enabled; }

    void setRadiationLevel(fc::RadiationMessage::Level lvl);

    virtual void handle(const fc::StorageMessage& msg);
    virtual void handle(const fc::SupplyMessage& msg);
    virtual void handle(const fc::StopMessage& msg);
    virtual void handle(const fc::StartMessage& msg);
private:
    Robot* robot;

    fc::Availability _storage;
    fc::Availability _supply;
    bool _enabled;
};
