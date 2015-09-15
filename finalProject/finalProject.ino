#include <BluetoothMaster.h>
#include "Message.h"
#include "BluetoothSystem.h"

const byte TEAM = 15;
BluetoothSystem bt(TEAM);
long last = 0;

void setup() {
    Serial.begin(9600);
    //pinMode(18, INPUT_PULLUP);
    //pinMode(19, INPUT_PULLUP);
    //Serial1.begin(115200);
    bt.init();
}

void loop() {
    bt.loop();
    long now = millis();
    if((now - last) > 2*1000) {
        Serial.print("Storage: ");
        Serial.print(bt.storage().tube1);
        Serial.print(" ");
        Serial.print(bt.storage().tube2);
        Serial.print(" ");
        Serial.print(bt.storage().tube3);
        Serial.print(" ");
        Serial.println(bt.storage().tube4);

        Serial.print("Supply: ");
        Serial.print(bt.supply().tube1);
        Serial.print(" ");
        Serial.print(bt.supply().tube2);
        Serial.print(" ");
        Serial.print(bt.supply().tube3);
        Serial.print(" ");
        Serial.println(bt.supply().tube4);
        last = now;
    }
}
