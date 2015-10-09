#pragma once

#include "Activity.h"
#include "PID.h"
#include "Motor.h"

class Robot;

enum Setpoint {
    UP, DOWN,
    OPEN = UP, CLOSE = DOWN,
    UNDEFINED,
};

class ArmSystem {
public:
    void init(Robot* robot);

    void start();

    void setArm(Setpoint set);
    void setGripper(Setpoint set);
    void setSlide(Setpoint set);
private:
    Robot* robot;
    
    class ArmIDA {
    public:
        void init(Robot* robot);

        void armAt(Setpoint set);
        void slideAt(Setpoint set);

        // No protection necessary on these variables, only ever modified
        // by activities (not interrupts)
        inline void setArm(Setpoint set) { arm_desired = set; }
        inline void setGripper(Setpoint set) { gripper_desired = set; }
        inline void setSlide(Setpoint set) { slide_desired = set; }

        inline Setpoint getArm() { return arm_desired; }
        inline Setpoint getGripper() { return gripper_desired; }
        inline Setpoint getSlide() { return slide_desired; }
    private:
        void done();

        Setpoint arm_desired;
        Setpoint arm_cur;

        Setpoint gripper_desired;
        Setpoint gripper_cur;

        Setpoint slide_desired;
        Setpoint slide_cur;

        Robot* robot;
    } arm_ida;

    class ArmActivity : public Activity {
    public:
        void init(ArmIDA* arm_ida);
        virtual void run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        ArmIDA* arm_ida;
        PID pid;
        Motor arm_motor;
    } arm_act;

    class SlideActivity : public Activity {
    public:
        void init(ArmIDA* arm_ida);
        virtual void run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        ArmIDA* arm_ida;
        Motor slide_motor;
    } slide_act;

    class GripperActivity : public Activity {
    public:
        void init(ArmIDA* arm_ida);
        virtual void run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:

        ArmIDA* arm_ida;
        Motor gripper_motor;
    } gripper_act;
};
