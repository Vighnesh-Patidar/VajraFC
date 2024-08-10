## THIS IS THE IMPLEMENTATION OF A FLIGHT CONTROLLER ON A TEENSY 4.1 MICROCONTROLLER BOARD
-Brief explanation of filesystems:-

![alt text](image.png)

 - the src directory contains all the necessary headers for control loop, PID, Kalman filter and sensor files
 - the autopilot.ino strings together all these files in a sensible order (given in the loop) and then publishes output to Motors
 - Motors controls the rotation velocity of the motors of quadcopter.
 - Calibrate file checks all the sensors at periodic intervals and logs the flight data.
