ME 405 Term Project
-----------
Introduction
This ROMI robot project was implemented and developed by Dane Carroll and Kassandra Martinez-Mejia as part of the ME 405 course's final project. The primary objective of this project was to design and develop a fully functional robot, named Romi, capable of navigating a track with distinct turns and obstacles, ultimately making its way back to the starting point. The project involved various stages, including hardware assembly, sensor integration, software development, and real-time testing. We utilized several key components such as the Romi Robot Kit including motors and encoders, QTR-8RC Reflectance Sensor Array, BNO055 IMU, and a bump sensor. Each component played a crucial role in enabling Romi to detect and respond to its environment. This development process provided hands-on experience in robotics, allowing us to apply theoretical knowledge in practical scenarios. Through iterative testing and debugging, we refined Romi's capabilities to achieve the best possible performance on the track. This document outlines the hardware and software implementations, provides detailed instructions for setting up and running the Romi robot, and reflects on the lessons learned throughout the project.

This ROMI uses:
* [Romi Robot Kit](https://www.pololu.com/product/3501)
* [QTR-8RC Reflectance Sensor Array](https://www.pololu.com/product/961)
* [BNO055 IMU](https://www.adafruit.com/product/2472)
* [Bump Sensor](https://www.pololu.com/product/1402)

![IMG_3082](https://github.com/user-attachments/assets/72e92c9e-c616-40de-8959-288f8fd9a484)

Video Demo (Best Official Attempt)
----------
Track Time around 46 seconds

[Romi Demo](https://youtu.be/s87TgsQ6EN8)

Line Following Demo (For Lab 0x04)
-----------
[Line Sensor Demo](https://youtube.com/shorts/gxOA3wK9QaQ)

Hardware Implementation
----------
**Line Sensor** 

We implemented our sensor based on the interfacing information provided in the Pololu website. In lab 4, the line sensor was able to read black and white datums based on the discharging times of the RC sensors. Then, we would normalize the readings with the values of the white and black datums. Lastly, we calculate the centroid of the sensor readings to then be able to adjust the motor efforts based on the error in centroid location. Unfortunately, the line sensor was not consistent enough to work properly for our final term project due to a multitude of resons, namely the variations in room lighting and the reflectivity of the game track surface. 


**Inertial Measurement Unit**

We used a BNO055 Inertial Measurement Unit (IMU) sensor to detect changes in orientation and measure headings, Euler angles, angular velocity, and the yaw rate. The BNO055 IMU integrates a 3-axis accelerometer, a 3-axis gyroscope, and a 3-axis magnetometer, providing comprehensive orientation data.

**Bump Sensor**

We implemented a bump sensor using a GPIO pin with a pull-up resistor to detect physical obstacles in the robot's path. The bump sensor was designed to provide feedback to the control system to prevent collisions and adjust the robot's movements accordingly. 

Wiring Diagram
-------------
![ME 405 Wiring Diagram](https://github.com/user-attachments/assets/890a9208-3beb-41d3-8dc9-746c81ee38b6)


Software Implementation
-------------
Throughout the quarter, we used Python classes to organize our code and shares and queues to update flags for specific tasks (motor, IMU, and data) on the scheduler. Also, we implement finite state machines (FSMs) for easy transistions to control Romi's movements. Due to the inconsistencies in results when using the line following sensor to complete the track, we decided to make a somewhat last minute pivot and instead break the track into a series of segmented vectors. In our final project implementation, we used a used these vectors by taking a target heading and drive distance using the IMU and encoder counts. This greatly simplified the code and made Romi's movements much more reliable, however with the limited time we were unable to finely tune the necessary encoder counts and angles in order to get Romi to consistently finish the track. 

![image](https://github.com/user-attachments/assets/18167bb8-2f02-4bd5-81ff-24647bcb6db6)


![image](https://github.com/user-attachments/assets/c524dbe2-5143-4d00-90ed-df60bb4af9d9)


**Motor Task**

We begin with initalizing the base effort of motors, the PID coefficents, encoders, and bump sensor. Once the IMU is calibrated, Romi begins to check for the current segment and the encoder's position and then transitions either to DRIVE, TURN, BUMP, BACKUP or END states. In the DRIVE state, we used a cascaded controller using proportional-integral control on the error between the target heading and the current heading reading, and proportional control on the error between left and right wheel velocities measured by the encoder. This allowed Romi to find the correct heading and drive as straight as possible. In TURN state, the heading error is again used to control the efforts in the left and right motors. Using proportional integral control and saturating the motor efforts, we can control the yaw rate and ensure that Romi is pointed in the right direction before beginning the next Drive segment. The BUMP and BACKUP states are used to detect and respond to the portion of the track where Romi interacts with the wall. In the END state, Romi comes to a stop. 

![image](https://github.com/user-attachments/assets/daca1d0c-b8b3-430f-8751-a08beb6d6eb8)


**IMU Task**

Initializes the IMU and calibrates the IMU's magnetometer, accelerometer and gyroscope. In the Initialization state, the IMU takes an average of ten headings to calibrate the starting heading. This starting heading is normalized and used as a new reference point of 0 degrees for the remainder of the program. By referencing from 0, programming turns is made much easier. After initializing, this task updates the current heading share for use in the motor task. 

![image](https://github.com/user-attachments/assets/19b0cd50-dcdd-4406-beb2-0f0f5d18e52f)


**Data Task**

The data task is responsible for collecting and displaying critical information about the robot's performance in real-time. This includes the current heading, distance traveled, and the velocity of the motors. 

![image](https://github.com/user-attachments/assets/c161276c-2ea2-44ad-8dfa-c0368376cdfe)



Instructions for Romi Robot
---------------
1. **Compile and Upload Code**: Compile the project code and upload it to the Romi's microcontroller using PuTTY. Ensure there are no errors during compilation and uploading.
2. **Initiate the Program**: Place the robot on the track and initiate the program.
3. **Calibrate IMU**: Rotate the Romi chassis around until the IMU sensor reads a value of 3 for the magnetometer, accelerometer, and gyroscope. This ensures accurate orientation data.
4. **Calibrate Starting Heading**: Point the Romi in the direction of the first desired vector to calibrate its starting heading. This step helps the IMU set a reference point for direction.
5. **Observe Romi's Behavior**: Place Romi on the track and observe its behavior as it runs through the track. 
6. **Adjust PID Constants**: Fine-tune the PID gains and needed encoder counts to optimize Romi's behavior and movements for the track. This may involve several iterations of testing and adjustment.


Conclusion
---------------
Even though Romi wasn’t able to complete the track, we were able to learn how to implement micropython on a robot using proportional-integral-derivative control, finite state machines (FSMs), and task schedulers. Throughout this project, we faced and overcame various challenges, including sensor integration, real-time data processing, and motor control. This experience has provided us with valuable insights into robotics programming and has allowed us to gain practical skills in hardware interfacing, software development, and system debugging. The knowledge and skills we have acquired here will be greatly applicable in our future engineering projects and careers.
