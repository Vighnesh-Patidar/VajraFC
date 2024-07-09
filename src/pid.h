#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd);
    void setTarget(float target);
    float compute(float measurement, float dt);

private:
    float kp, ki, kd;
    float target;
    float integral;
    float previousError;
};

#endif
