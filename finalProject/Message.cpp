#include "Message.h"
#include "stdio.h"

namespace fc {

const uint8_t Message::start_delim = 0x5F;

Type Message::type() {
    return _type;
}

int Message::encode(uint8_t* buffer, size_t len) {
    if (len < 6) return -1;
    int data_len = fill_data(buffer+5, len-6);
    if (data_len < 0) return -1;

    buffer[0] = start_delim;
    buffer[1] = data_len + 5;
    buffer[2] = _type;
    buffer[3] = src();
    buffer[4] = dst();

    uint8_t checksum = 0xFF;
    for(int i = 1; i < data_len+5; ++i) {
        checksum -= buffer[i];
    }

    buffer[data_len+5] = checksum;
    return data_len+6;
}

Message Message::decode(uint8_t* buffer, size_t len) {
    Message msg;
    decode(msg, buffer, len);
    return msg;
}

void Message::decode(Message& msg, uint8_t* buffer, size_t len) {
    msg._type = INVALID;
    if (len < 6) return;
    if (buffer[0] != start_delim) return;
    int msg_len = buffer[1]+1;
    if (len < msg_len) return;
    Type type = static_cast<Type>(buffer[2]);
    Address src = buffer[3];
    Address dst = buffer[4];

    uint8_t checksum = 0xFF;
    for(int i = 1; i < msg_len; ++i) {
        checksum -= buffer[i];
    }

    if(buffer[msg_len-1] == checksum) return;

    msg._type = type;

    switch (type) {
        case STORAGE_AVAILABILITY: {
            msg.msg.storageMessage.src = src;
            msg.msg.storageMessage.dst = dst;
            msg.msg.storageMessage.availability.byte = buffer[5];
            break;
        }
        case SUPPLY_AVAILABILITY: {
            msg.msg.supplyMessage.src = src;
            msg.msg.supplyMessage.dst = dst;
            msg.msg.supplyMessage.availability.byte = buffer[5];
            break;
        }
        case RADIATION_ALERT: {
            msg.msg.radiationMessage.src = src;
            msg.msg.radiationMessage.dst = dst;
            // TODO: Add validity check?
            msg.msg.radiationMessage.level = static_cast<RadiationMessage::Level>(buffer[5]);
            break;
        }
        case STOP: {
            msg.msg.stopMessage.src = src;
            msg.msg.stopMessage.dst = dst;
            break;
        }
        case START: {
            msg.msg.startMessage.src = src;
            msg.msg.startMessage.dst = dst;
            break;
        }
        case ROBOT_STATUS: {
            msg.msg.statusMessage.src = src;
            msg.msg.statusMessage.dst = dst;
            // TODO: Add validity check?
            msg.msg.statusMessage.movement_status = static_cast<StatusMessage::MovementStatus>(buffer[5]);
            msg.msg.statusMessage.gripper_status = static_cast<StatusMessage::GripperStatus>(buffer[6]);
            msg.msg.statusMessage.operation_status = static_cast<StatusMessage::OperationStatus>(buffer[7]);
            break;
        }
        case HEARTBEAT: {
            msg.msg.heartbeatMessage.src = src;
            msg.msg.heartbeatMessage.dst = dst;
            break;
        }
        default:
            msg._type = INVALID;
    }
}

int Message::fill_data(uint8_t* buffer, size_t len) {
    switch(_type) {
        case STORAGE_AVAILABILITY:
            return msg.storageMessage.fill_data(buffer, len);
        case SUPPLY_AVAILABILITY:
            return msg.supplyMessage.fill_data(buffer, len);
        case RADIATION_ALERT:
            return msg.radiationMessage.fill_data(buffer, len);
        case STOP:
            return msg.stopMessage.fill_data(buffer, len);
        case START:
            return msg.startMessage.fill_data(buffer, len);
        case ROBOT_STATUS:
            return msg.statusMessage.fill_data(buffer, len);
        case HEARTBEAT:
            return msg.heartbeatMessage.fill_data(buffer, len);
        default:
            return 0;
    }
}

void Message::handleWith(MessageHandler& v) {
    switch(_type) {
        case STORAGE_AVAILABILITY:
            msg.storageMessage.handleWith(v);
            break;
        case SUPPLY_AVAILABILITY:
            msg.supplyMessage.handleWith(v);
            break;
        case RADIATION_ALERT:
            msg.radiationMessage.handleWith(v);
            break;
        case STOP:
            msg.stopMessage.handleWith(v);
            break;
        case START:
            msg.startMessage.handleWith(v);
            break;
        case ROBOT_STATUS:
            msg.statusMessage.handleWith(v);
            break;
        case HEARTBEAT:
            msg.heartbeatMessage.handleWith(v);
            break;
        default:
            break;
    }
}

Message::Message() : _type(INVALID) {}

Message::Message(const Message& other) : _type(other._type) {
    switch(other._type) {
        case STORAGE_AVAILABILITY:
            msg.storageMessage = other.msg.storageMessage;
            break;
        case SUPPLY_AVAILABILITY:
            msg.supplyMessage = other.msg.supplyMessage;
            break;
        case RADIATION_ALERT:
            msg.radiationMessage = other.msg.radiationMessage;
            break;
        case STOP:
            msg.stopMessage = other.msg.stopMessage;
            break;
        case START:
            msg.startMessage = other.msg.startMessage;
            break;
        case ROBOT_STATUS:
            msg.statusMessage = other.msg.statusMessage;
            break;
        case HEARTBEAT:
            msg.heartbeatMessage = other.msg.heartbeatMessage;
            break;
        default:
            _type = INVALID;
            break;
    }
}

Message::Message(const StorageMessage& msg) {
    this->_type = STORAGE_AVAILABILITY;
    this->msg.storageMessage = msg;
}

Message::Message(const SupplyMessage& msg) {
    this->_type = SUPPLY_AVAILABILITY;
    this->msg.supplyMessage = msg;
}

Message::Message(const RadiationMessage& msg) {
    this->_type = RADIATION_ALERT;
    this->msg.radiationMessage = msg;
}

Message::Message(const StopMessage& msg) {
    this->_type = STOP;
    this->msg.stopMessage = msg;
}

Message::Message(const StartMessage& msg) {
    this->_type = START;
    this->msg.startMessage = msg;
}

Message::Message(const StatusMessage& msg) {
    this->_type = ROBOT_STATUS;
    this->msg.statusMessage = msg;
}

Message::Message(const HeartbeatMessage& msg) {
    this->_type = HEARTBEAT;
    this->msg.heartbeatMessage = msg;
}

// ---------------------------------------------------------------------
// ----------------    Field Control Message Types    ------------------
// ---------------------------------------------------------------------
Type StorageMessage::type()   { return STORAGE_AVAILABILITY; }
Type SupplyMessage::type()    { return SUPPLY_AVAILABILITY; }
Type RadiationMessage::type() { return RADIATION_ALERT; }
Type StopMessage::type()      { return STOP; }
Type StartMessage::type()     { return START; }
Type StatusMessage::type()    { return ROBOT_STATUS; }
Type HeartbeatMessage::type() { return HEARTBEAT; }

// ---------------------------------------------------------------------
// ---------------------    Visitor handleWithors    -----------------------
// ---------------------------------------------------------------------
void StorageMessage::handleWith(MessageHandler& v)   { v.handle(*this); }
void SupplyMessage::handleWith(MessageHandler& v)    { v.handle(*this); }
void RadiationMessage::handleWith(MessageHandler& v) { v.handle(*this); }
void StopMessage::handleWith(MessageHandler& v)      { v.handle(*this); }
void StartMessage::handleWith(MessageHandler& v)     { v.handle(*this); }
void StatusMessage::handleWith(MessageHandler& v)    { v.handle(*this); }
void HeartbeatMessage::handleWith(MessageHandler& v) { v.handle(*this); }


// ---------------------------------------------------------------------
// ---------------------    Message Encoders    ------------------------
// ---------------------------------------------------------------------
int StorageMessage::fill_data(uint8_t* buffer, size_t len) {
    if (len < 1) return -1;
    buffer[0] = availability.byte;
    return 1;
}

int SupplyMessage::fill_data(uint8_t* buffer, size_t len) {
    if (len < 1) return -1;
    buffer[0] = availability.byte;
    return 1;
}

int RadiationMessage::fill_data(uint8_t* buffer, size_t len) {
    if (len < 1) return -1;
    buffer[0] = level;
    return 1;
}

int StopMessage::fill_data(uint8_t* buffer, size_t len) {
    return 0;
}

int StartMessage::fill_data(uint8_t* buffer, size_t len) {
    return 0;
}

int StatusMessage::fill_data(uint8_t* buffer, size_t len) {
    if (len < 3) return -1;
    buffer[0] = movement_status;
    buffer[1] = gripper_status;
    buffer[2] = operation_status;
    return 3;
}

int HeartbeatMessage::fill_data(uint8_t* buffer, size_t len) {
    return 0;
}

// ---------------------------------------------------------------------
// --------------------    Src/Dst extractors    -----------------------
// ---------------------------------------------------------------------
Address Message::src() {
    switch(_type) {
        case STORAGE_AVAILABILITY:
            return msg.storageMessage.src;
        case SUPPLY_AVAILABILITY:
            return msg.supplyMessage.src;
        case RADIATION_ALERT:
            return msg.radiationMessage.src;
        case STOP:
            return msg.stopMessage.src;
        case START:
            return msg.startMessage.src;
        case ROBOT_STATUS:
            return msg.statusMessage.src;
        case HEARTBEAT:
            return msg.heartbeatMessage.src;
        default:
            return 0;
    }
}

Address Message::dst() {
    switch(_type) {
        case STORAGE_AVAILABILITY:
            return msg.storageMessage.dst;
        case SUPPLY_AVAILABILITY:
            return msg.supplyMessage.dst;
        case RADIATION_ALERT:
            return msg.radiationMessage.dst;
        case STOP:
            return msg.stopMessage.dst;
        case START:
            return msg.startMessage.dst;
        case ROBOT_STATUS:
            return msg.statusMessage.dst;
        case HEARTBEAT:
            return msg.heartbeatMessage.dst;
        default:
            return 0;
    }
}

} // namespace fc

// Message examples
int test() {
    uint8_t buf[7];
    {
        fc::HeartbeatMessage msg;
        msg.src = 10;
        msg.dst = 0;

        const int len = 6;
        uint8_t knownEncoding[len] = { 0x5F, 0x05, 0x07, 0x0A, 0x00, 0xE9 };

        int bytes = fc::Message(msg).encode(buf, 7);
        if (bytes != len) { printf("Message length did not match for message 1\n"); }

        bool same = true;
        for (int i = 0; i < bytes; ++i) {
            if (buf[i] != knownEncoding[i]) same = false;
            printf("0x%x ", buf[i]);
        }
        printf("\n");
        if (!same) { printf("Encoded message did not match for message 1\n"); }

        same = true;
        bytes = fc::Message::decode(buf, 7).encode(buf, 7);
        if (bytes != len) { printf("Reencoded message length did not match for message 1\n");  }
        for (int i = 0; i < bytes; ++i) {
            if (buf[i] != knownEncoding[i]) same = false;
            printf("0x%x ", buf[i]);
        }
        printf("\n");
        if (!same) { printf("Reencoded message did not match for message 1\n"); }
    }

    {
        fc::StorageMessage msg;
        msg.src = 0;
        msg.dst = 0;
        msg.availability.byte = 0;
        msg.availability.tubes.tube1 = true;
        msg.availability.tubes.tube3 = true;
        int bytes = fc::Message(msg).encode(buf, 7);
        for (int i = 0; i < bytes; ++i) {
            printf("0x%x ", buf[i]);
        }
        printf("\n");
        bytes = fc::Message::decode(buf, 7).encode(buf, 7);
        for (int i = 0; i < bytes; ++i) {
            printf("0x%x ", buf[i]);
        }
        printf("\n");
    }

    {
        fc::StopMessage msg;
        msg.src = 0;
        msg.dst = 5;
        int bytes = fc::Message(msg).encode(buf, 7);
        for (int i = 0; i < bytes; ++i) {
            printf("0x%x ", buf[i]);
        }
        printf("\n");
        bytes = fc::Message::decode(buf, 7).encode(buf, 7);
        for (int i = 0; i < bytes; ++i) {
            printf("0x%x ", buf[i]);
        }
        printf("\n");
    }
    return 0;
}
