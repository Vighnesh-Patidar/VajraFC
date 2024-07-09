#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "ekf.h"
#include "pid.h"
#include "motorcontrol.h"

// Pin definitions for LSM9DS1 using hardware SPI
#define LSM9DS1_XGCS 24  // Chip select for accelerometer/gyroscope
#define LSM9DS1_MCS 25   // Chip select for magnetometer

// Pin definitions for BMP280 using hardware SPI
#define BMP_CS 10

// Initialize LSM9DS1 sensor using hardware SPI
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// Initialize BMP280 sensor using hardware SPI
Adafruit_BMP280 bmp(BMP_CS);

// Initialize EKF
ExtendedKalmanFilter ekfAccelX;
ExtendedKalmanFilter ekfAccelY;
ExtendedKalmanFilter ekfAccelZ;
ExtendedKalmanFilter ekfGyroX;
ExtendedKalmanFilter ekfGyroY;
ExtendedKalmanFilter ekfGyroZ;
ExtendedKalmanFilter ekfMagX;
ExtendedKalmanFilter ekfMagY;
ExtendedKalmanFilter ekfMagZ;
ExtendedKalmanFilter ekfTemp;
ExtendedKalmanFilter ekfPressure;

// Initialize PID Controllers for each axis
PID pidRoll(1.0, 0.0, 0.0);   // PID parameters for roll
PID pidPitch(1.0, 0.0, 0.0);  // PID parameters for pitch
PID pidYaw(1.0, 0.0, 0.0);    // PID parameters for yaw

void setupLSM9DS1() {
    // Set accelerometer range and data rate
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
    
    // Set magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    
    // Setup gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void setupBMP280() {
    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
        while (1);
    }
    
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial monitor to open
    }

    Serial.println("LSM9DS1 & BMP280 Data Read Demo");

    // Initialize LSM9DS1 sensor with hardware SPI
    if (!lsm.begin()) {
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        while (1);
    }
    Serial.println("Found LSM9DS1 9DOF sensor");

    // Initialize BMP280 sensor with hardware SPI
    setupBMP280();

    // Setup sensor parameters for LSM9DS1
    setupLSM9DS1();

    // Initialize EKF for each sensor axis and parameter
    ekfAccelX.initialize(0.0, 1.0, 0.1, 0.1);
    ekfAccelY.initialize(0.0, 1.0, 0.1, 0.1);
    ekfAccelZ.initialize(0.0, 1.0, 0.1, 0.1);

    ekfGyroX.initialize(0.0, 1.0, 0.1, 0.1);
    ekfGyroY.initialize(0.0, 1.0, 0.1, 0.1);
    ekfGyroZ.initialize(0.0, 1.0, 0.1, 0.1);

    ekfMagX.initialize(0.0, 1.0, 0.1, 0.1);
    ekfMagY.initialize(0.0, 1.0, 0.1, 0.1);
    ekfMagZ.initialize(0.0, 1.0, 0.1, 0.1);

    ekfTemp.initialize(0.0, 1.0, 0.1, 0.1);
    ekfPressure.initialize(0.0, 1.0, 0.1, 0.1);

    // Set initial PID targets (for example, level flight)
    pidRoll.setTarget(0.0);
    pidPitch.setTarget(0.0);
    pidYaw.setTarget(0.0);

    // Initialize motor control
    initializeMotors();
}

void loop() {
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Delta time in seconds
    lastTime = currentTime;

    // Read LSM9DS1 sensor data
    lsm.read();

    // Get LSM9DS1 sensor events
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    // Apply EKF to accelerometer data
    ekfAccelX.predict(0.0);
    ekfAccelX.update(a.acceleration.x);

    ekfAccelY.predict(0.0);
    ekfAccelY.update(a.acceleration.y);

    ekfAccelZ.predict(0.0);
    ekfAccelZ.update(a.acceleration.z);

    // Apply EKF to gyroscope data
    ekfGyroX.predict(0.0);
    ekfGyroX.update(g.gyro.x);

    ekfGyroY.predict(0.0);
    ekfGyroY.update(g.gyro.y);

    ekfGyroZ.predict(0.0);
    ekfGyroZ.update(g.gyro.z);

    // Apply EKF to magnetometer data
    ekfMagX.predict(0.0);
    ekfMagX.update(m.magnetic.x);

    ekfMagY.predict(0.0);
    ekfMagY.update(m.magnetic.y);

    ekfMagZ.predict(0.0);
    ekfMagZ.update(m.magnetic.z);

    // Read BMP280 sensor data
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa (hectopascals)

    // Compute PID control outputs
    float rollCommand = pidRoll.compute(ekfGyroX.getState(), dt);
    float pitchCommand = pidPitch.compute(ekfGyroY.getState(), dt);
    float yawCommand = pidYaw.compute(ekfGyroZ.getState(), dt);

    // Set motor speeds based on PID commands
    setMotorSpeeds(rollCommand, pitchCommand, yawCommand);

    // Print sensor and control data to Serial Monitor
    Serial.print("Roll Command: "); Serial.println(rollCommand);
    Serial.print("Pitch Command: "); Serial.println(pitchCommand);
    Serial.print("Yaw Command: "); Serial.println(yawCommand);

    Serial.println();

    delay(100); // Adjust delay as needed
}
