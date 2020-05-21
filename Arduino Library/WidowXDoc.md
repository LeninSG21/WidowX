# Arduino Library

The WidowX Arduino Library was created to simplify the operation of the [WidowX Robot Arm](https://www.trossenrobotics.com/widowxrobotarm) of [Trossen Robotics](https://www.trossenrobotics.com/) when controlled with the [Arbotix-M Robocontroller](https://learn.trossenrobotics.com/arbotix). It is based on the [ax12.h library](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h). Also, inside some of its functions, it uses the Matrix object as defined in [BasicLinearAlgebra.h](https://github.com/tomstewart89/BasicLinearAlgebra). Hence, it requires installation in order to work properly.

## Installation

First, you’ll need to follow the [ArbotiX-M Robocontroller Getting Started Guide](https://learn.trossenrobotics.com/arbotix/arbotix-quick-start.html) and test the correct connectivity with your WidowX arm. Be mindful of the Arduino version that is being installed, since the documentation specifies that Arduino 1.0.6 is the only one that works with the ArbotiX.

Once you complete this guide, open the Arduino IDE and access to the library manager via **Sketch > Include Library > Manage Libraries** and look for _Basic Linear Algebra_. Select the library by Tom Stewart and install it.

Afterwards, go to the [WidowX Library repo](https://github.com/LeninSG21/WidowX), download it as a zip and extract it. Then, go to the folder ArduinoLibrary and copy the WidowX folder into the Arduino/libraries directory. Open again the Arduino IDE and the WidowX library should appear under the available libraries at **Sketch > Include Library > Contributed Libraries**.

Also, you should see in the same space the Bioloid and BasicLinearAlgebra libraries. That means that everything is installed properly and that the arm is ready to be used.

To confirm the installation, open the file _Arduino Library/Examples/testArm.ino._ The arm should move to rest position, then home, then center and finally rest position again.

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
uint8_t id[] = {1,2,3,4,5,6}
```
