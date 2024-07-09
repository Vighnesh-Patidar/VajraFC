#ifndef EKF_H
#define EKF_H

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    void initialize(float initialState, float initialCovariance, float processNoise, float measurementNoise);
    void predict(float controlInput);
    void update(float measurement);
    float getState();

private:
    float state;
    float covariance;
    float processNoise;
    float measurementNoise;
};

#endif
