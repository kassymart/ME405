ME 405 Term Project
-----------
This ROMI robot project was implemented and developed by Dane Carrol and Kassandra Martinez-Mejia. The goal for the project is
the ROMI robot must complete a track with distinct turns and obstacles as it makes its way to the start. 

This ROMI uses:
* [Romi Robot Kit](https://www.pololu.com/product/3501)
* [QTR-8RC Reflectance Sensor Array](https://www.pololu.com/product/961)
* [BNO055 IMU](https://www.adafruit.com/product/2472)

Video Demo
----------


Hardware Implementation
----------
**Line Sensor**
[discuss logic + calibration of the line sensor]

**IMU**
[discuss logic + calibration of the IMU]
Software Implementation

**PID controllers**
[discuss logic + calibration of motor and encoders]

Software Implementation
-------------
Throughout the quarter, we used Python classes to organize our code and used shares and queues to send flags for specific tasks
necessary. Each task also uses FSMs (finite state machine) to ensure we know which state ROMI is "reacting" to as we test and
debug ROMI's movements. 

Instructions for Romi Robot
---------------
1. Calibrate line sensor array on all-white surface and all-black surface.
2. Calibrate IMU by rotating the Romi chassis around until it reads 3 operating modes: sys, gyroscope, and magnetometer.
3. Observe Romi's behavior as it is reading black lines on the track.
4. Adjust PID constants (Kp, Ki, Kd) to ensure that Romi's behavior and movements are best suited for the track.
5. Adjust the IMU and make sure Romi is configured with the initial heading.
6. Compile and upload the code onto the microcontroller.
7. Place the robot on the track and begin the line-following mode.
**add image on hardware setup of Romi**
