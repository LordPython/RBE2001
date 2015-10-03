class PID {
public:
    inline void init(float p, float i, float d, int deadband = 0) {
        this->p = p; this->i = i; this->d = d; this->deadband = deadband;
    }

    inline int calc(int error) {
        if (abs(error)<deadband) return 0;
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
