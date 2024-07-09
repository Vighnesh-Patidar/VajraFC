#include "ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() : state(0), covariance(1), processNoise(1), measurementNoise(1) {}

void ExtendedKalmanFilter::initialize(float initialState, float initialCovariance, float processNoise, float measurementNoise) {
    state = initialState;
    covariance = initialCovariance;
    this->processNoise = processNoise;
    this->measurementNoise = measurementNoise;
}

void ExtendedKalmanFilter::predict(float controlInput) {
    // State prediction
    state += controlInput;

    // Covariance prediction
    covariance += processNoise;
}

void ExtendedKalmanFilter::update(float measurement) {
    // Measurement update
    float kalmanGain = covariance / (covariance + measurementNoise);
    state = state + kalmanGain * (measurement - state);
    covariance = (1 - kalmanGain) * covariance;
}

float ExtendedKalmanFilter::getState() {
    return state;
}
