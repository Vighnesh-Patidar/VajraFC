#include <Arduino.h>
#include "motorcontrol.h"

// Define motor control pins
const int motorFrontLeft = 9;
const int motorFrontRight = 10;
const int motorRearLeft = 11;
const int motorRearRight = 12;

void initializeMotors() {
    pinMode(motorFrontLeft, OUTPUT);
    pinMode(motorFrontRight, OUTPUT);
    pinMode(motorRearLeft, OUTPUT);
    pinMode(motorRearRight, OUTPUT);

    // Initialize motors to stop position (1500 us pulse width for most ESCs)
    analogWrite(motorFrontLeft, 1500);
    analogWrite(motorFrontRight, 1500);
    analogWrite(motorRearLeft, 1500);
    analogWrite(motorRearRight, 1500);
}

void setMotorSpeeds(float rollCommand, float pitchCommand, float yawCommand) {
    int baseSpeed = 1500; // Base speed (neutral position)

    int speedFL = constrain(baseSpeed + rollCommand + pitchCommand - yawCommand, 1000, 2000); // Front-left motor
    int speedFR = constrain(baseSpeed - rollCommand + pitchCommand + yawCommand, 1000, 2000); // Front-right motor
    int speedRL = constrain(baseSpeed + rollCommand - pitchCommand + yawCommand, 1000, 2000); // Rear-left motor
    int speedRR = constrain(baseSpeed - rollCommand - pitchCommand - yawCommand, 1000, 2000); // Rear-right motor

    analogWrite(motorFrontLeft, speedFL);
    analogWrite(motorFrontRight, speedFR);
    analogWrite(motorRearLeft, speedRL);
    analogWrite(motorRearRight, speedRR);
}
