#include <Servo.h>
#include <TimerOne.h>
#include <QTRSensors.h>

#include "Robot.h"

#define DEBUG

const byte TEAM = 15;
Robot robot;

void setup() {
//#ifdef DEBUG
    Serial.begin(9600);
//#endif
    robot.init(TEAM);
}

void loop() {
    int b = Serial.read();
    switch(b) {
        case 'n':
          robot.planner.next();
          break;
        default:
          break;
    }
    robot.runNext();
}
