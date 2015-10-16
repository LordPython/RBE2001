#pragma once

#include "Activity.h"
#include "PID.h"
#include "Motor.h"

// Can't include robot.h (circular dependency)
// so just declare robot here.
class Robot;

/**
 * Possible setpoints for the arm, slide and gripper.
 * UP/OPEN, and CLOSE/DOWN. UNDEFINED is used to represent
 * the current location for inital state.
 **/
enum Setpoint {
    UP, DOWN,
    OPEN = UP, CLOSE = DOWN,
    UNDEFINED,
};

/**
 * @brief System to control the arm, slide and gripper
 **/
class ArmSystem {
public:
    /**
     * @brief Initialize the subsystem
     **/
    void init(Robot* robot);
    /**
     * @brief Schedule the activities to control the arm, slide and gripper
     **/
    void start();
    /**
     * @brief Set the setpoint for the arm control activity
     **/
    void setArm(Setpoint set);
    /**
     * @brief Set the setpoint for the gripper control activity
     **/
    void setGripper(Setpoint set);
    /**
     * @brief Set the setpoint for the slide control activity
     **/
    void setSlide(Setpoint set);
private:
    //! Reference to the robot object
    Robot* robot;

    /**
     * @breif Facilitates communication between the activites
     * within the arm system and the rest of the system.
     *
     * (IDA stands for Intercommunication Data Area).
     **/
    class ArmIDA {
    public:
        /**
         * @brief Initialize the object
         * @param robot Pointer to the robot object
         **/
        void init(Robot* robot);

        /**
         * @breif Update current position of arm
         * Called by the arm activity indicating that the
         * arm is at the given setpoint
         * @param set Setpoint indicating current position of the arm
         **/
        void armAt(Setpoint set);
        /**
         * @breif Update current position of slide
         * Called by the slide activity indicating that the
         * slide is at the given setpoint
         * @param set Setpoint indicating current position of the slide
         **/
        void slideAt(Setpoint set);

        // No protection necessary on these variables, only ever modified
        // by activities (not interrupts)
        /**
         * @breif Set desired arm position
         * @param set desired arm position
         **/
        inline void setArm(Setpoint set) { arm_desired = set; }
        /**
         * @breif Set desired gripper position
         * @param set desired gripper position
         **/
        inline void setGripper(Setpoint set) { gripper_desired = set; }
        /**
         * @breif Set desired slide position
         * @param set desired slide position
         **/
        inline void setSlide(Setpoint set) { slide_desired = set; }

        /**
         * @breif Gets arm setpoint
         * @return arm setpoint
         **/
        inline Setpoint getArm() { return arm_desired; }
        /**
         * @breif Gets gripper setpoint
         * @return gripper setpoint
         **/
        inline Setpoint getGripper() { return gripper_desired; }
        /**
         * @breif Gets slide setpoint
         * @return slide setpoint
         **/
        inline Setpoint getSlide() { return slide_desired; }
    private:
        void done();

        //! Desired position for arm
        Setpoint arm_desired;
        //! Current arm position
        Setpoint arm_cur;
        //! Desired position for gripper
        Setpoint gripper_desired;
        //! Current gripper position
        Setpoint gripper_cur;
        //! Desired position for slide
        Setpoint slide_desired;
        //! Current slide position
        Setpoint slide_cur;
        //! Pointer to robot system
        Robot* robot;
    } arm_ida;

    /**
     * @breif Control loop activity to run the arm
     **/
    class ArmActivity : public Activity {
    public:
        /**
         * @breif Initialize activity
         * @param arm_ida pointer to the control object
         **/
        void init(ArmIDA* arm_ida);

        virtual bool run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        //! Pointer to control object
        ArmIDA* arm_ida;
        //! PID loop object for controlling the arm
        PID pid;
        //! Arm motor
        Motor arm_motor;
    } arm_act;

    /**
     * @breif Control loop activity to run the slide
     **/
    class SlideActivity : public Activity {
    public:
        void init(ArmIDA* arm_ida);
        virtual bool run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        //! Pointer to control object
        ArmIDA* arm_ida;
        //! Slide motor
        Motor slide_motor;
    } slide_act;

    /**
     * @breif Control loop activity to run the gripper
     **/
    class GripperActivity : public Activity {
    public:
        void init(ArmIDA* arm_ida);
        virtual bool run();
        virtual Priority priority() { return CONTROL_LOOP; }
    private:
        //! Pointer to control object
        ArmIDA* arm_ida;
        //! Gripper servo
        Motor gripper_motor;
    } gripper_act;
};
