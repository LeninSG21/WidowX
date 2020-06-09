# Arduino Library Examples

In this directory you will find some useful codes to implement with your WidowX Robot Arm right away. Take a look to how they work and modify them to suit your needs!

## Test Arm

The `testArm.ino` file is ideal to check if the installation was done properly. Just load it into your ArbotiX and watch the arm move from rest, to home, to center and back to rest. Also, check the Serial Monitor to observe some useful messages, like the operation voltage of the arm.

## How to Use

The `HowToUse.ino` file allows you to interact with some of the functions of the WidowX library through serial commands. You can send the arm to the preset positions (Home, Center and Rest) or move each servo individually either by giving the desired angle (as seen from the coordinate system described for this library, where the center position of each servo is equal to 0°) or by giving the position, as described in the appropriate dynamixel datasheet (from 0-4095 for MX-28 and MX-64 and from 0-1023 fro the AX-12a). Also, you can see the effect of relaxing the servos and activating the torque. Make sure to grab the arm or to have it in rest position before calling the relax command. Finally, you can see a demonstration of the different inverse kinematics functions. Look at the serial terminal to read the inverse kinematics function that is being used and the point and rotation that is desired to achieve. Then look the arm do it. This is a good way to understand the different effects each function has and by looking at the code you can learn how to implement it.

## Move With Controller

The `MoveWithController.ino` file is designed to receive a message via the serial port to move the WidowX arm with a controller. This code only interprets the message received and sends the appropriate information to the WidowX library to move the arm. It does not care who sends the message and how it build it. Therefore, you can use this code with any controller and button mapping you want, as long as you follow the message structure defined next.
