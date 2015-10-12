#pragma once

/**
 * Convenience class for performing a PID loop
 **/
class PID {
public:
    /**
     * Initialize the PID with the given constants
     * @param p Proportional term
     * @param i Integral term
     * @param d Derivative term
     * @param deadband Deadband within which the system is considered to be at the setpoint
     **/
    inline void init(float p, float i, float d, int deadband = 0) {
        this->p = p; this->i = i; this->d = d; this->deadband = deadband;
    }

    /**
     * Calculate the PID output given the error
     * @param error current error
     * @return PID output
     **/
    inline int calc(int error) {
        if (abs(error)<deadband) {
            // If we're within the deadband, 
            // zero out accumulated error.
            accum = 0;
            return 0;
        }
        float result = p*error + i*accum + d*(error-last_error);
        accum += error;
        last_error = error;
        return result;
    }
private:
    float p, i, d;
    long accum;
    int last_error, deadband;
};
