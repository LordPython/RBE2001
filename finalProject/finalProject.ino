#include <LiquidCrystal.h>
#include <Servo.h>
#include <TimerOne.h>
#include <QTRSensors.h>

#include "Robot.h"
#include "Ports.h"
#include "Activity.h"

#define DEBUG

const byte TEAM = 15;
Robot robot;

bool button_hit = false;

void buttonISR() {
    button_hit = true;
}

class CalibrateActivity : public Activity {
    virtual void run() {
        robot.nav.calibrate();
    }

    virtual Priority priority() { return MAIN; }
} calibrate_act;

void setup() {
//#ifdef DEBUG
    Serial.begin(9600);
//#endif
    robot.init(TEAM);
    pinMode(START_BUTTON_PORT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(START_BUTTON_PORT), buttonISR, FALLING);
    robot.schedule(calibrate_act);
    button_hit = false;
}

void loop() {
    if (button_hit) {
        Serial.println("Starting!");
        button_hit = false;
        robot.deschedule(calibrate_act);
        robot.start();
    }
    
    robot.runNext();
}
