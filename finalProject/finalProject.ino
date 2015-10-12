#include <TimerThree.h>
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
    virtual bool run() {
        robot.nav.calibrate();

        if (button_hit) {
            button_hit = false;
            robot.start();
            return true;
        }
        
        return false;
    }

    virtual Priority priority() { return MAIN; }
} calibrate_act;

void setup() {
    Serial.begin(9600);
    robot.init(TEAM);
    pinMode(START_BUTTON_PORT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(START_BUTTON_PORT), buttonISR, FALLING);
    robot.schedule(calibrate_act);
    button_hit = false;
}

void loop() {
    robot.runNext();
}
