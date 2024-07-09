#include "pid.h"

PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), target(0), integral(0), previousError(0) {}

void PID::setTarget(float target) {
    this->target = target;
}

float PID::compute(float measurement, float dt) {
    float error = target - measurement;
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}
