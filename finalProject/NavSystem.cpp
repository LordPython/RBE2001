#include "NavSystem.h"
#include "Robot.h"

void NavSystem::init(Robot* robot) {
    this->robot = robot;
    nav_act.init(this);
    robot->schedule(nav_act);

    pinMode(FRONT_BUMP_PORT, INPUT_PULLUP);
    left.init(LEFT_DRIVE_PORT);
    right.init(RIGHT_DRIVE_PORT);

    current = START;
    desired = START;
    current_command.type = DONE;
}

void NavSystem::go(Location loc) {
    desired = loc;
    nav_act.resetStateTime();

    if (current != START) {
        // Every navigation (except at the very start)
        // starts by backing up and turning around
        commands.insert(Command(BACK_UP));
        commands.insert(Command(TURN_AROUND));
    }

    switch(current) {
    case START:
        break;
    case REACTOR_A:
        switch(desired) {
        case SUPPLY_1:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_2:
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_3:
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_4:
            commands.insert(Command(FOLLOW_COUNT, 4));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case STORAGE_1:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case STORAGE_2:
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case STORAGE_3:
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case STORAGE_4:
            commands.insert(Command(FOLLOW_COUNT, 4));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        }
        break;
    case REACTOR_B:
        switch(desired) {
        case SUPPLY_1:
            commands.insert(Command(FOLLOW_COUNT, 4));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_2:
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_3:
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_4:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case STORAGE_1:
            commands.insert(Command(FOLLOW_COUNT, 4));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case STORAGE_2:
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case STORAGE_3:
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case STORAGE_4:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        }
        break;
    case SUPPLY_1:
    case SUPPLY_2:
    case SUPPLY_3:
    case SUPPLY_4:
        switch(desired) {
        case REACTOR_A:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case REACTOR_B:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        }
        break;
    case STORAGE_1:
        switch(desired) {
        case SUPPLY_1:
            break;
        case SUPPLY_2:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_3:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_4:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        }
        break;
    case STORAGE_2:
        switch(desired) {
        case SUPPLY_1:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_2:
            break;
        case SUPPLY_3:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        case SUPPLY_4:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        }
        break;
    case STORAGE_3:
        switch(desired) {
        case SUPPLY_1:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_2:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_3:
            break;
        case SUPPLY_4:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            break;
        }
        break;
    case STORAGE_4:
        switch(desired) {
        case SUPPLY_1:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 3));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_2:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 2));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_3:
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_RIGHT));
            commands.insert(Command(FOLLOW_COUNT, 1));
            commands.insert(Command(FORWARD));
            commands.insert(Command(TURN_LEFT));
            break;
        case SUPPLY_4:
            break;
        }
        break;
    }

    // Every navigation ends at a limit
    commands.insert(Command(FOLLOW_LIMIT));
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
    pid.init(0.02, 0.0, 0.07);
    unsigned long now = millis();
    lastLineTime = now;
    lastStateTime = now;
}

void NavSystem::NavActivity::resetStateTime() {
    lastStateTime = millis();
    done_pause = false;
}

void NavSystem::NavActivity::run() {
    if(!nav->robot->status.enabled()) {
        nav->stop();
        return;
    }
    unsigned int sensorValues[NUM_SENSORS];
    unsigned long now = millis();
    unsigned long timeSinceLastState = now-lastStateTime;

    if (!done_pause) {
        if(timeSinceLastState >= 100) {
            done_pause = true;
            lastStateTime = now;
            timeSinceLastState = 0;
        } else {
            nav->stop();
            return;
        }
    }

    switch (nav->current_command.type) {
    case BACK_UP: {
        int back_up_time = 400;
        if (nav->current == REACTOR_B) {
            back_up_time = 2000;
        }
        if (timeSinceLastState > back_up_time) {
            nav->stop();
            nav->next();
            lastStateTime = now;
            done_pause = false;
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
            done_pause = false;
        } else {
            nav->drive(-50,50);
        }
        break;
    case TURN_AROUND: {
        int error = nav->qtrrc8.readLine(sensorValues) - 3500;
        if (timeSinceLastState > 1100) {
            int diff = pid.calc(error);
            nav->drive(diff, diff);
            
            if(error < 300) {
                nav->stop();
                nav->next();
                lastStateTime = now;
                done_pause = false;
            }
        } else {
            nav->drive(90,90);
        }
        break;
    }
    case TURN_LEFT: {
        int position = nav->qtrrc8.readLine(sensorValues);
        long sum = 0;
        for (int i = 0; i < NavSystem::NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }
        //if(sum < 400) {
        //if(now-lastStateTime > 1000) {
        
        if (timeSinceLastState > 600 && ((position > 3000 && position < 4000))) {
            nav->stop();
            nav->next();
            lastStateTime = now;
            done_pause = false;
        } else {
            nav->drive(50,50);
        }
        break;
    }
    case TURN_RIGHT: {
        int position = nav->qtrrc8.readLine(sensorValues);
        long sum = 0;
        for (int i = 0; i < NavSystem::NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }
        //if(sum < 400) {
        //if(timeSinceLastState > 1000) {
        if (timeSinceLastState > 700 && ((position > 3000 && position < 4000))) {
            nav->stop();
            nav->next();
            lastStateTime = now;
            done_pause = false;
        } else {
            nav->drive(-50,-50);
        }
        break;
    }
    case FOLLOW_LIMIT:
        if (digitalRead(FRONT_BUMP_PORT)) {
            followLine();
        } else {
            nav->current = nav->desired;
            lastStateTime = now;
            done_pause = false;
            nav->next();
            nav->stop();
        }
        break;
    case FOLLOW_COUNT: {
        bool intersection = followLine();
        if (intersection) intersections++;
        if (intersections >= nav->current_command.intersections) {
            intersections = 0;
            nav->stop();
            nav->next();
            lastStateTime = now;
            done_pause = false;
        }
        break;
    }
    case DONE:
        nav->stop();
        break;
    }
}


bool NavSystem::NavActivity::followLine() {
    unsigned long now = millis();

    unsigned sensorValues[NavSystem::NUM_SENSORS];
    long position = nav->qtrrc8.readLine(sensorValues);

    bool intersection = false;

    if (now - lastLineTime > 500) {
        long sum = 0;
        for (int i = 0; i < NavSystem::NUM_SENSORS; ++i) {
            sum += sensorValues[i];
        }
        if (sum > 900*8) {
            intersection = true;
            lastLineTime = now;
        }
    }

    int diff = pid.calc(position - 3500);
    const int leftBaseSpeed = 40;
    const int rightBaseSpeed = 55;

    nav->drive(diff - leftBaseSpeed, diff + rightBaseSpeed);

    return intersection;
}
