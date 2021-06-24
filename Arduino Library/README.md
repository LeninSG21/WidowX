# Arduino Library

The WidowX Arduino Library was created to simplify the operation of the [WidowX Robot Arm](https://www.trossenrobotics.com/widowxrobotarm) of [Trossen Robotics](https://www.trossenrobotics.com/) when controlled with the [Arbotix-M Robocontroller](https://learn.trossenrobotics.com/arbotix). It is based on the [ax12.h library](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h). 

For simplicity, each motor's angle is labeled Q1, Q2, Q3, Q4, Q5 and Q6. So whenever you see **Qn**, know that it is reffering to the angle of the motor.

## Installation

First, you’ll need to follow the [ArbotiX-M Robocontroller Getting Started Guide](https://learn.trossenrobotics.com/arbotix/arbotix-quick-start.html) and test the correct connectivity with your WidowX arm. Be mindful of the Arduino version that is being installed, since the documentation specifies that Arduino 1.0.6 is the only one that works with the ArbotiX.

Afterwards, go to the [WidowX Library repo](https://github.com/LeninSG21/WidowX), download it as a zip and extract it. Then, go to the folder ArduinoLibrary and copy the WidowX folder into the Arduino/libraries directory. Open again the Arduino IDE and the WidowX library should appear under the available libraries at **Sketch > Include Library > Contributed Libraries**.

Also, you should see in the same space the Bioloid library. That means that everything is installed properly and that the arm is ready to be used.

To confirm the installation, open the file [**Arduino Library > Examples > testArm.ino**](https://github.com/LeninSG21/WidowX/blob/master/Arduino%20Library/Examples/testArm/testArm.ino). The arm should move to rest position, then home, then center and finally rest position again. Open the Serial terminal at 115,200 bps to check the messages from the test.

## WidowX Files

### Poses.h

This file defines poses and loads them into the memory of the microcontroller. Each pose is an array of uint16_t of length six. Each element represents the position of each of the motors, which for the first four motors goes from 0 to 4095 (MX-28 and MX-64) and for the last two goes from 0 to 1023 (AX-12a). If any constant pose is to be added, this file is an appropriate place to do it.

### Keywords.txt

This file indicates the Arduino IDE which words should be highlighted when using the library. In this case, the KEYWORD1 is assigned to “WidowX” and the public functions have the KEYWORD2 flag. To understand it better, take a look at the [Writing a Library for Arduino](https://www.arduino.cc/en/Hacking/LibraryTutorial) tutorial.

### WidowX.h

This is the header files that contains the definitions of the functions, variables and constants used.

### WidowX.cpp

This file implements the code for the functions defined in the header file. It also defines some global variables used throughout the codes that do not belong to the WidowX class.

## Functions

The following methods belong to the WidowX class. They will be presented in order, separating them into public and private, and subclassified into role. However, before going into the explanation of the code, the difference between _id_ and _idx_ must be addressed to understand the functions.

### id vs idx

When working with Dynamixel motors, each one of them must have a unique id by which it will be controlled. Usually, for the WidowX arm, the ids of the motors go from 1 to 6, where the id 1 corresponds to the motor at the base and the id 6 to the gripper. However, each person can set the ids of their motors any way they want, so addressing the first motor with the id 1 could not always work. Hence, the WidowX library does not work with the id, instead, it uses what we will call the idx. This idx the absolute numbering of the motors from 0 to 5, where the idx 0 corresponds to the first motor and the idx 5 to the last motor, which is the gripper. So, when you want to move or get information from a specific motor, you should give the idx that will always be the same no matter the id you give to said motor.

Nonetheless, the ax12.h library does require the id of the motor to move it accordingly. That is why the id array is created inside the WidowX class. This array maps the id of a motor to its idx. For example, let’s say we have the following id array:

```c
uint8_t id[] = {1,2,3,4,5,6}
```

This array is saying that the first motor has an id of 1, the second an id of 2, and so on until the last motor, the gripper, with an id of 6. This is the standard configuration, which is why the id array is initialized to these values by the constructor. However, let’s imagine that you need to set the ids of the motors from 16 to 21. Then, the id array would look like this:

```c
uint8_t id[] = {16,17,18,19,20,21}
```

If you needed to move a motor to a specific position, let’s say the third motor, you would have to use the idx, which is 2 for the third motor. What the function will do internally is that it will take the element of the id array corresponding to the idx. Look at the following pseudocode:

```c
void someFunction(int idx) //assume idx = 2 for the example
{
  int actualID = id[idx];
  //If id[] = {1,2,3,4,5,6}, id[idx] = 3
  //If id[] = {16,17,18,19,20,21}, id[idx] = 18
  GetPosition(actualID);//use of ax12.h library that requires id
}
```

The next picture shows the motors numbered. Remember that the idx values are fixed, while the id can be changed by the user. Both the idx and the common value of the id are shown for each motor. For the functions that are described next, remember that they require the idx.

![Image of Numbered Motors](https://github.com/LeninSG21/WidowX/blob/master/NumberedMotors.png)

## Public

### Initializers

#### WidowX()

> Constructor of the class. Needs to be called first to create an instance. It assigns values to some constants and fills the id array from 1 to 6.

#### void init(uint8_t relax)

> This is NOT a required function in order to work properly. It sends the arm to rest position. It also checks the voltage with the function checkVoltage(). If relax != 0, then the torque is disabled after it reaches the rest position.

### ID Handlers

#### void setId(int idx, int newId)

> Sets the id of the specified motor by its idx. For example, if the gripper motor has an id of 21, then the id array must be updated for the class to work properly. Thus, this function is called as follows: setId(5,21). The idx is not changed whatsoever and to move the desired motor the same idx should be given in the other functions.

#### int getId(int idx)

> Gets the id of the specified motor as saved in the id array of the class. Useful to check that the id assigned to the motor in the id array corresponds to the actual id given to the Dynamixel.

### Preloaded Poses

#### void moveCenter()

> Moves the arm to the center pose, which is when all the motors are sent to its center position, hence forming an upside L, in the default time of two seconds. This is equal to setting all the motors to 0°, as seen from the kinematic analysis done for this library. It loads the pose from memory, as defined in poses.h. Updates point once it’s done.

#### void moveHome()

> Moves the arm to the home pose defined by Bioloid in the default time of two seconds. It loads the pose from memory, as defined in poses.h. Fig. 1 shows the arm in home pose. Updates point once it’s done.

#### void moveRest()

> Moves the arm to the rest pose in the default time of two seconds. When in rest, the arm lies on itself. This is a safe position to disable torque. It loads the pose from memory, as defined in poses.h. Updates point once it’s done.
> void moveToPose(const unsigned int \*pose)
> Moves the arm to the given pose in the default time of two seconds. For it to work, the given pose must be loaded into memory. A way to define your own pose would be to place it in poses.h. Updates point once it’s done.

### Get Information

#### void checkVoltage()

> Checks that the voltage values are adequate for the robotic arm. If it is below 10V, it remains in a loop until the voltage increases. This is to prevent damage to the arm. Also, it sends through the serial port of the ArbotiX some information about the voltage, which can be then seen by the serial monitor or received with another interface connected to the ArbotiX serial.

#### void getCurrentPosition()

> Updates the position of all the motors of the arm. It calls getServoPosition()
> void getCurrentPosition(uint8_t until_idx)
> Updates the position of the motors from the first to the given by the parameter. It calls getServoPosition(). It is useful when the position of some motors needs to be updated. For example, to update the position of motors from 0 to 3, you would do getCurrentPosition(3).

#### int getServoPosition(int idx)

> This function calls the GetPosition function from the ax12.h library. However, it was seen that in some cases the value returned was -1. Hence, it made the arm to move drastically, which, represents a hazard to those around and the arm itself. That is why this function checks if the returned value is -1. If it is, reads the current position of the specified motor until the value is not -1. Also, for every iteration, the delay between lectures varies from 5 to 45ms, to give time to the motor to respond appropriately. Due to this check conditions, this function is preferred over the readPose() function from the BioloidController.h library. The problem is that it might cause a significant delay if the motors keep returning -1. Nonetheless, it is better to have this delay than to risk the arm’s integrity.
> This function updates the following arrays at the given index (idx): current_position, the current_angle with the function positionToAngle(), and the float_position with the current_position. It returns the current_position of the specified motor.

#### float getServoAngle(int idx)

> This function calls the getServoPosition() function at the given idx to update the current_angle array and returns the angle at the index specified.

#### void getPoint(float \*p)

> Calls the private function updatePoint() to load the current point into the class variable point. Then, it saves the point values [x,y,z] into the pointer p. This should be an array of at least length three.

### Torque

#### void relaxServos()

> This function disables the torque of all the motors via Relax() from ax12.h and sets the global flag of isRelaxed to true. WARNING: when using it be careful of the arm's position; otherwise it can be damaged if it impacts too hard on the ground or with another object.

#### void torqueServos()

> This function enables the torque of every motor. It does not alter their current positions.

### Move Servo

#### void moveServo2Angle(int idx, float angle)

> Sends the specified motor to the desired angle in radians. The angle and the directions of turn are the ones specified in the forward kinematics analysis done for this library. That is, angle 0 rad is when the motor is at its center position. It moves the servo smoothly.

#### void moveServo2Position(int idx, int pos);

> Sets the position of the specified motor to the position given. Be wary of the ranges, since it does not validate the given position. For MX-28 and MX-64, it goes from 0 to 4095, and for AX-12a from 0 to 1023. It moves the servo smoothly.

#### void moveGrip(int close)

> Closes or opens the gripper (Q6 | idx = 5): close = 0  open, close = 1  close in steps of 10. Use it inside a loop with a delay to control the smoothness of the turn. Ideal for movement with control or key that is being sent while it is pressed.

#### void setServo2Position(int idx, int position)

> Unlike the function moveServo2Position(), this one does not move the servo smoothly. It only sends the servo to the desired position. It basically just wraps the SetPosition() function from ax12.h to be used with idx instead of index. Be careful not to write values out of range or to make a huge jump in position—for example, having the motor at position 50 and moving it to 750.

#### void moveServoWithSpeed(int idx, int speed, long initial_time)

> This is a function designed to work with a joystick or controller. To move the motor with speed control, use this function inside a loop. At the beginning of the loop, the initial_time is set—for example, with the function millis() in Arduino. Then, you receive the speed value, which represents an analog signal. You send those two parameters, along with the specified motor, to this function. With this function, you would expect the motor to move 90° per second when the speed equals 255. Velocity can be a positive or negative value.

### Move Arm

#### void movePointWithSpeed(int vx, int vy, int vz, int vg, long initial_time)

> This function is designed to work with a joystick or controller. It moves the origin of the grippers coordinate system. To move the arm with speed control, use this function inside a loop. At the beginning of the loop, the initial_time is set—for example, with the function millis() in Arduino. The function receives four different velocities:
>
> - vx: velocity to move the x-coordinate as seen from the base of the robot
> - vy: velocity to move the y-coordinate as seen from the base of the robot
> - vz: velocity to move the z-coordinate as seen from the base of the robot
> - vg: velocity to move the angle of the gripper as seen from the base of the robot

> This function was designed considering that speed values range from [-127,127] for vx, vy and vz and [-255,255] for vg. Higher values are mathematically possible, but will make the arm move faster, so be cautious. This function offers a better controlling experience for machines.

#### void moveArmWithSpeed(int vx, int vy, int vz, int vg, long initial_time)

> This function also moves the arm when controlled with a joystick or controller, just as movePointWithSpeed(). However, while the other function moves according to the desired translation and rotation of the origin of the grippers coordinate system, this function moves the arm to the front or back with vx, it turns it around with vy and with vy it goes up and down. This function was designed considering that speed values range from [-127,127] for vx, vy and vz and [-255,255] for vg. Higher values are mathematically possible, but will make the arm move faster, so be cautious. This function offers a better controlling experience for human operators.

#### void moveArmQ4(float Px, float Py, float Pz)

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot. It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function only affects Q1, Q2 and Q3. It interpolates the step using a cubic interpolation with the default time. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor.

#### void moveArmQ4(float Px, float Py, float Pz, int time)

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot. It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function only affects Q1, Q2 and Q3. It interpolates the step using a cubic interpolation with the given time in milliseconds. If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.

#### void moveArmGamma(float Px, float Py, float Pz, float gamma)

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. It interpolates the step using a cubic interpolation with the default time. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor.

#### void moveArmGamma(float Px, float Py, float Pz, float gamma, int time)

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. It interpolates the step using a cubic interpolation with the given time in milliseconds. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor.

#### void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd)

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matrix of 3x3, the gripper's rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be the same. Thus, it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation with the default time. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor. Rd is a Matrix given as a bidimensial float array
#### void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd, int time);

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, and with the desired rotation of the coordinate system of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matrix of 3x3, the gripper's rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be the same. Thus, it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation with the given time in milliseconds. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor. Rd is a Matrix given as a bidimensial float array

#### void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase);

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system of the gripper, as seen from the base of the robot. It uses getIK_RdBase. This function affects Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation with the default time. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor.

#### void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase, int time);

> Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system of the gripper, as seen from the base of the robot. It uses getIK_RdBase. This function affects Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation with the given time in milliseconds. If there is no solution for the IK, the arm does not move, and a message is printed into the serial monitor.

#### void rotz(float angle, Matrix<3, 3> &Rz)

> This function saves a rotation matrix in Z by the given angle in rads into the Matrix object Rz.

#### void roty(float angle, Matrix<3, 3> &Ry)

> This function saves a rotation matrix in Y by the given angle in rads into the Matrix object Ry.

#### void rotx(float angle, Matrix<3, 3> &Rx)

> This function saves a rotation matrix in X by the given angle in rads into the Matrix object Rx.
