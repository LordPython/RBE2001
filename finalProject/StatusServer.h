#pragma once

#include "Message.h"
#include <LiquidCrystal.h>

class Robot;

/**
 * Possible radiation levels
 **/
enum RadiationLevel {
    HIGH_RAD, LOW_RAD, NO_RAD,
};

/**
 * System responsible for keeping track of
 * current field and robot status (radiation level,
 * etc.)
 **/
class StatusServer : public MessageHandler {
public:
    //! Initialize the lcd
    StatusServer() : lcd(40, 41, 42, 43, 44, 45) {}
    //! Initialize the system
    void init(Robot* robot);

    /**
     * Get the current state storage tubes
     **/
    inline Availability storage() { return _storage; }
    /**
     * Get the current state supply tubes
     **/
    inline Availability supply() { return _supply; }
    /**
     * Check if the robot is enabled
     * @return whether the robot is currently enabled
     **/
    inline bool enabled() { return _enabled; }

    /**
     * Notify the system of the current radiation level
     * This will send out the appropriate bluetooth message
     * and write an indication to the lcd.
     **/
    void setRadiationLevel(RadiationLevel lvl);

    // Message handlers
    virtual void handle(const StorageMessage& msg);
    virtual void handle(const SupplyMessage& msg);
    virtual void handle(const StopMessage& msg);
    virtual void handle(const StartMessage& msg);
private:

    //! Print the storage status to the lcd
    void printStorage();
    //! Print the supply status to the lcd
    void printSupply();
    //! Print the current radiation status to the lcd
    void printRadiation();
    //! Print whether or not the robot is enabled
    void printStopped();

    Robot* robot;

    LiquidCrystal lcd;

    Availability _storage;
    Availability _supply;
    RadiationLevel lvl;
    bool _enabled;
};
