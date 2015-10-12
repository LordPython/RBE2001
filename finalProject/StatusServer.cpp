#include "StatusServer.h"
#include "Robot.h"

void StatusServer::init(Robot* robot) {
    this->robot = robot;
    lvl = NO_RAD;
    _enabled = true;

    lcd.begin(16,2);
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    printStopped();
    printRadiation();
    printSupply();
    printStorage();
}

void StatusServer::setRadiationLevel(RadiationLevel l) {
    lvl = l;
    if (lvl != NO_RAD) {
        RadiationMessage msg;
        msg.src = robot->addr;
        msg.dst = 0;
        msg.level = lvl == HIGH_RAD ? RadiationMessage::NEW : RadiationMessage::SPENT;
        robot->bluetooth.send(msg);
    }
    printRadiation();
}

void StatusServer::handle(const StorageMessage& msg) {
    _storage = msg.availability.tubes;
    printStorage();
}

void StatusServer::handle(const SupplyMessage& msg) {
    _supply = msg.availability.tubes;
    printSupply();
}

void StatusServer::handle(const StopMessage& msg) {
    Serial.println("Stop");
    Serial.flush();
    _enabled = false;
    printStopped();
}

void StatusServer::handle(const StartMessage& msg) {
    Serial.println("Start");
    Serial.flush();
    _enabled = true;
    printStopped();
}

void StatusServer::printRadiation() {
    const int SIZE = 10;
    lcd.setCursor(0,1);
    char lcdMsg[SIZE];
    char* str;
    switch(lvl) {
    default:
    case NO_RAD:
        str = "NONE";
        break;
    case HIGH_RAD:
        str = "NEW";
        break;
    case LOW_RAD:
        str = "SPENT";
        break;
    }
    snprintf(lcdMsg, SIZE, "Rad:%5s", str);
    lcd.print(lcdMsg);
}

void StatusServer::printStorage() {
    lcd.setCursor(0,0);
    const int SIZE = 9;
    char lcdMsg[SIZE];
    snprintf(lcdMsg, SIZE, "St:%1d%1d%1d%1d ", _storage.tube1, _storage.tube2, _storage.tube3, _storage.tube4);
    lcd.print(lcdMsg);
    Serial.println(lcdMsg);
}

void StatusServer::printSupply() {    
    lcd.setCursor(8,0);
    const int SIZE = 8;
    char lcdMsg[SIZE];
    snprintf(lcdMsg, SIZE, "Su:%1d%1d%1d%1d ",  _supply.tube1, _supply.tube2, _supply.tube3, _supply.tube4);
    lcd.print(lcdMsg);
    Serial.println(lcdMsg);
}

void StatusServer::printStopped() {   
    lcd.setCursor(11,1); 
    const int SIZE = 6;
    char lcdMsg[SIZE];
    const char* str = _enabled ? "Run" : "Stop";
    snprintf(lcdMsg, SIZE, "%s ",  str);
    lcd.print(lcdMsg);
    Serial.println(lcdMsg);
}

