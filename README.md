ME 405 Term Project
-----------
This ROMI robot project was implemented and developed by Dane Carroll and Kassandra Martinez-Mejia. The goal for the project is
the ROMI robot must complete a track with distinct turns and obstacles as it makes its way back to the starting point. 

This ROMI uses:
* [Romi Robot Kit](https://www.pololu.com/product/3501)
* [QTR-8RC Reflectance Sensor Array](https://www.pololu.com/product/961)
* [BNO055 IMU](https://www.adafruit.com/product/2472)

Video Demo (Best Attempt)
----------
Track Time around 46 seconds

[Romi Demo](https://youtu.be/s87TgsQ6EN8)

Hardware Implementation
----------
**Line Sensor** 

We implemented our sensor based on the interfacing information provided in the Pololu website. In lab 4, the line sensor was able to read black and white datums based on the discharging times of the RC sensors. Then, normalize the readings with the values of the white and black datums. Lastly, calculate the centroid of the sensor readings. The line sensor did not work properly for our final term project due to the inconsistent readings during calibration. 

**IMU**
[discuss logic + calibration of the IMU]
Software Implementation

**PID controllers**
[discuss logic + calibration of motor and encoders]

**Bump Sensor**
We implemented a bumper sensor using a GPIO pin that has a pull-up resistor. Also, we created a Python class for the bump sensor to update the pressed state of the sensor and adjust for debounce timing using a timer. 

**Wiring Diagram**
![ME 405 Wiring Diagram](https://github.com/user-attachments/assets/762dbc34-67e7-4c59-b7b7-1a2298e9efaa)



Software Implementation
-------------
Throughout the quarter, we used Python classes to organize our code and used shares and queues to send flags for specific tasks
necessary. Each task also uses FSMs (finite state machine) to ensure we know which state ROMI is "reacting" to as we test and
debug ROMI's movements. 
[probably add more here]

Instructions for Romi Robot
---------------
1. Calibrate IMU by rotating the Romi chassis around until it reads 3 operating modes: sys, gyroscope, and magnetometer.
2. Calibrate Romi by swaying Romi back and forth. 
3. Observe Romi's behavior as it is reading black lines on the track.
4. Adjust PID constants (Kp, Ki, Kd) to ensure that Romi's behavior and movements are best suited for the track.
5. Adjust the IMU and make sure Romi is configured with the initial heading.
6. Compile and upload the code onto the microcontroller.
7. Place the robot on the track and begin the line-following mode.
![IMG_3082](https://github.com/user-attachments/assets/72e92c9e-c616-40de-8959-288f8fd9a484)

