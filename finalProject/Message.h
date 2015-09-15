#pragma once

#include <stdint.h>
#include <stddef.h>

namespace fc {

class Message;
class StorageMessage;
class SupplyMessage;
class RadiationMessage;
class StopMessage;
class StartMessage;
class StatusMessage;
class HeartbeatMessage;

struct Availability {
    bool tube1 : 1;
    bool tube2 : 1;
    bool tube3 : 1;
    bool tube4 : 1;
};

enum Type : uint8_t {
    INVALID = 0,
    STORAGE_AVAILABILITY = 1,
    SUPPLY_AVAILABILITY = 2,
    RADIATION_ALERT = 3,
    STOP = 4,
    START = 5,
    ROBOT_STATUS = 6,
    HEARTBEAT = 7
};

class MessageHandler {
public:
    virtual void handle(const StorageMessage& msg) {}
    virtual void handle(const SupplyMessage& msg) {}
    virtual void handle(const RadiationMessage& msg) {}
    virtual void handle(const StopMessage& msg) {}
    virtual void handle(const StartMessage& msg) {}
    virtual void handle(const StatusMessage& msg) {}
    virtual void handle(const HeartbeatMessage& msg) {}
};

typedef uint8_t Address;

class StorageMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
    // Tube availability
    union {
        uint8_t byte;
        Availability tubes;
    } availability;
};

class SupplyMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
    // Tube availability
    union {
        uint8_t byte;
        Availability tubes;
    } availability;
};

class RadiationMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
    // Radiation Level
    enum Level : uint8_t {
        SPENT = 0x2C,
        NEW   = 0xFF,
    } level;
};

class StopMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
};

class StartMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
};

class StatusMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
    // Movement status
    enum MovementStatus : uint8_t {
        STOPPED = 1,
        TELEOP = 2,
        AUTO = 3,
    } movement_status;
    // Gripper Status
    enum GripperStatus : uint8_t {
        HAS_ROD = 1,
        NO_ROD = 2,
    } gripper_status;
    // Operation status
    enum OperationStatus : uint8_t {
        GRIP_IN_PROGRESS = 1,
        RELEASE_IN_PROGRESS = 2,
        DRIVING_TO_REACTOR = 3,
        DRIVING_TO_STORAGE = 4,
        DRIVING_TO_SUPPLY = 5,
        IDLE = 6,
    } operation_status;
};

class HeartbeatMessage {
private:
    friend class Message;
    int fill_data(uint8_t* buffer, size_t len);
public:
    Type type();
    void handleWith(MessageHandler& v);
    // Source address
    Address src;
    // Destination address
    Address dst;
};

class Message {
public:
    Address src();
    Address dst();
    Type type();
    // Encode a message to a buffer
    int encode(uint8_t* buffer, size_t len);
    // Decode a message from a buffer
    static Message decode(uint8_t* buffer, size_t len);
    static void decode(Message& msg, uint8_t* buffer, size_t len);
    // handleWith a visitor
    void handleWith(MessageHandler& v);
    // Copy Constructor
    Message(const Message& msg);
    // Construct from concrete message types
    Message(const StorageMessage& msg);
    Message(const SupplyMessage& msg);
    Message(const RadiationMessage& msg);
    Message(const StopMessage& msg);
    Message(const StartMessage& msg);
    Message(const StatusMessage& msg);
    Message(const HeartbeatMessage& msg);
private:
    static const uint8_t start_delim;
    
    int fill_data(uint8_t* buffer, size_t len);

    Message();

    Type _type;
    union {
        StorageMessage storageMessage;
        SupplyMessage supplyMessage;
        RadiationMessage radiationMessage;
        StopMessage stopMessage;
        StartMessage startMessage;
        StatusMessage statusMessage;
        HeartbeatMessage heartbeatMessage;
    } msg;
};

}
