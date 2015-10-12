#include <TimerThree.h>
#include "NavSystem.h"
#include "Robot.h"

unsigned long timeEnabledCounter = 0;

unsigned long timeEnabled() {
    uint8_t saveSREG = SREG;
    cli();
    unsigned long t = timeEnabledCounter;
    SREG = saveSREG;
    return t;
}

void timer3() {
    extern Robot robot;
    if (robot.status.enabled()) {
        timeEnabledCounter++;
    }
}

void NavSystem::init(Robot* robot) {
    this->robot = robot;
    nav_act.init(this);

    pinMode(FRONT_BUMP_PORT, INPUT_PULLUP);
    left.init(LEFT_DRIVE_PORT);
    right.init(RIGHT_DRIVE_PORT);

    current_pos = START;
    desired_pos = START;
    current_dir = WEST;
    current_command.type = DONE;

    timeEnabledCounter = 0;
    Timer3.initialize(1000);
    Timer3.attachInterrupt(timer3);
}

void NavSystem::start() {
    robot->schedule(nav_act);
}

void NavSystem::go(Vector new_pos) {
    nav_act.resetStateTime();

    Vector tracking_pos = current_pos;
    Vector tracking_dir = current_dir;

    if (current_pos == START) {
        commands.insert(Command(FOLLOW_LIMIT));
        desired_dir = WEST;
        desired_pos = new_pos;
        next();
        return;
    }
    
    commands.insert(Command(BACK_UP));
    commands.insert(Command(TURN_AROUND));
    tracking_dir = -tracking_dir;
    commands.insert(Command(FOLLOW_COUNT, 1));
    tracking_pos += tracking_dir;

    int dist_along_center = new_pos.x - tracking_pos.x;
    int turn_dir = tracking_dir.cross(Vector{dist_along_center,0});

    if (turn_dir > 0) {
        commands.insert(Command(FORWARD));
        commands.insert(Command(TURN_LEFT));
        tracking_dir.rotate(LEFT);
    } else if (turn_dir < 0) {
        commands.insert(Command(FORWARD));
        commands.insert(Command(TURN_RIGHT));
        tracking_dir.rotate(RIGHT);
    }

    if (abs(dist_along_center) > 0) {
        commands.insert(Command(FOLLOW_COUNT, abs(dist_along_center)));
    }

    turn_dir = tracking_dir.cross(Vector{0, new_pos.y});

    if (turn_dir > 0) {
        commands.insert(Command(FORWARD));
        commands.insert(Command(TURN_LEFT));
        tracking_dir.rotate(LEFT);
    } else if (turn_dir < 0) {
        commands.insert(Command(FORWARD));
        commands.insert(Command(TURN_RIGHT));
        tracking_dir.rotate(RIGHT);
    }


    // Every navigation ends at a limit
    commands.insert(Command(FOLLOW_LIMIT));

    desired_dir = tracking_dir;
    desired_pos = new_pos;

    next();
}

void NavSystem::calibrate() {
    qtrrc8.calibrate();
}

void NavSystem::drive(int left, int right) {
    this->left.write(left);
    this->right.write(right);
}

void NavSystem::next() {
    //current = desired;
    bool available = commands.pop(current_command);

    if(!available) {
        current_command.type = DONE;
        robot->planner.next();
    } else {
        Serial.print("Popping command of type: ");
        Serial.println(current_command.type);
    }
}

void NavSystem::NavActivity::init(NavSystem* nav) {
    this->nav = nav;
    pid.init(0.03, 0.0, 0.07);
    unsigned long now = timeEnabled();
    lastStateTime = now;
    lastLineTime = now;
}

void NavSystem::NavActivity::resetStateTime() {
    lastStateTime = timeEnabled();
}

bool NavSystem::NavActivity::run() {
    unsigned long now = timeEnabled();
    unsigned long timeSinceLastState = now-lastStateTime;
    unsigned long timeSinceLastLine = now-lastLineTime;
    // Always read line sensor (so we know where the line was last seen
    unsigned int sensorValues[NUM_SENSORS];
    long position = nav->qtrrc8.readLine(sensorValues);

    // Check if we've crossed an intersection
    // This is checked by seening if the average of the sensor values is >900
    if (timeSinceLastLine > 500) {
        long sum = 0;
        for (int i = 0; i < NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }
        if (sum > 900*NUM_SENSORS) {
            // If we've found and intersection, and are currently counting intersections,
            // count it
            if (nav->current_command.type == FOLLOW_COUNT) --nav->current_command.data;
            // Reset time since we last saw a line.
            lastLineTime = now;
        }
    }

    // If the robot is not enabled, stop driving, and return
    if(!nav->robot->status.enabled()) {
        nav->stop();
        return false;
    } 

    switch (nav->current_command.type) {
    case BACK_UP: {
        int back_up_time = 400;
        if (nav->current_pos == REACTOR_B) {
            back_up_time = 2000;
        }
        if (timeSinceLastState > back_up_time) {
            nav->stop();
            nav->next();
            lastStateTime = now;
        } else {
            nav->drive(50,-50);
        }
        break;
    }
    case FORWARD:
        if (timeSinceLastState > 200) {
            nav->stop();
            nav->next();
            lastStateTime = now;
        } else {
            nav->drive(-50,50);
        }
        break;
    case TURN_AROUND: /*{
        int error = position - 3500;
        if (timeSinceLastState > 1100) {
            int diff = pid.calc(error);
            nav->drive(diff, diff);
            
            if(error < 300) {
                nav->stop();
                nav->next();
                timeSinceLastState = 0;
            }
        } else {
            nav->drive(90,90);
        }
        break;
    }*/
    case TURN_LEFT: {
        long sum = 0;
        for (int i = 0; i < NavSystem::NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }

        if (sum < 100*NUM_SENSORS) { nav->current_command.data = 1; }
      
        if ((nav->current_command.data == 1) && ((position > 3000 && position < 4000))) {
            nav->stop();
            nav->next();
            lastStateTime = now;
        } else {
            nav->drive(60,60);
        }
        break;
    }
    case TURN_RIGHT: {
        long sum = 0;
        for (int i = 0; i < NavSystem::NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }

        if (sum < 100*NUM_SENSORS) { nav->current_command.data = 1; }
        if ((nav->current_command.data == 1) && ((position > 3000 && position < 4000))) {
            nav->stop();
            nav->next();
            lastStateTime = now;
        } else {
            nav->drive(-60,-60);
        }
        break;
    }
    case FOLLOW_LIMIT:
        if (digitalRead(FRONT_BUMP_PORT)) {
            followLine(position);
        } else {
            nav->current_pos = nav->desired_pos;
            nav->current_dir = nav->desired_dir;
            lastStateTime = now;
            nav->next();
            nav->stop();
        }
        break;
    case FOLLOW_COUNT:
        followLine(position);
        if (nav->current_command.data <= 0) {
            nav->stop();
            nav->next();
            lastStateTime = now;
        }
        break;
    case DONE:
        nav->stop();
        break;
    }
    return false;
}


void NavSystem::NavActivity::followLine(long position) {
    int diff = pid.calc(position - 3500);
    const int leftBaseSpeed = 50;
    const int rightBaseSpeed = 50;

    nav->drive(diff - leftBaseSpeed, diff + rightBaseSpeed);
}
