#include "StatusServer.h"
#include "Robot.h"

void StatusServer::init(Robot* robot) {
    this->robot = robot;
    _enabled = true;
}

void StatusServer::setRadiationLevel(fc::RadiationMessage::Level lvl) {
    fc::RadiationMessage msg;
    msg.src = robot->addr;
    msg.dst = 0;
    msg.level = lvl;
    robot->bluetooth.send(msg);

    // TODO: Flash LEDs!
}

void StatusServer::handle(const fc::StorageMessage& msg) {
    _storage = msg.availability.tubes;
}

void StatusServer::handle(const fc::SupplyMessage& msg) {
    _supply = msg.availability.tubes;
}

void StatusServer::handle(const fc::StopMessage& msg) {
    Serial.println("Stop");
    Serial.flush();
    _enabled = false;
    // TODO: Flash LEDs!
}

void StatusServer::handle(const fc::StartMessage& msg) {
    Serial.println("Start");
    Serial.flush();
    _enabled = true;
    // TODO: Flash LEDs!
}
