# WidowX

WidowX.h is an Arduino library designed to simplify the operation of the [WidowX Robot Arm](https://www.trossenrobotics.com/widowxrobotarm) of [Trossen Robotics](https://www.trossenrobotics.com/) when controlled with the [Arbotix-M Robocontroller](https://learn.trossenrobotics.com/arbotix). It is based on [BioloidController.h](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/BioloidController.h) and [ax12.h](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h). Inside some of its functions it uses the Matrix object as defined in [BasicLinearAlgebra.h](https://github.com/tomstewart89/BasicLinearAlgebra). Hence, it requires installation in order to work properly.

This repository also contains the analysis of the kinematics, both direct and inverse, of this robot arm in a Matlab live script. It is highly recommended to take a look to said live script to understand better both the arm and the analysis that gave birth to this library.

## Installation

Download the WidowX folder and copy it into the Arduino/libraries folder. It should appear in the **include library** section. You'll need to include also the ax12.h&mdash;as seen in the [ArbotiX-M Robocontroller Getting Started Guide](https://learn.trossenrobotics.com/arbotix/arbotix-quick-start.html)&mdash;and the BasicLinearAlgebra.h libraries.

## Documentation

### Understanding WidowX.h

This library controls the movement of the robot by using the inverse kinematics analysis done by the author&mdash;this analysis is explained thoroughly in the Matlab live script [WidowX_Kinematics](https://github.com/LeninSG21/WidowX/blob/master/Matlab/WidowX_Kinematics.mlx)&mdash;. To do so there are four different ways to move the arm to a desired position as seen from the base of the robot. This position will be a point in the 3D space. To move the arm to said point while maintaining the current angle of the wrist, which gives the function.

### Understanding the WidowX

In this section the kinematics of the WidowX Rotbot Arm will be described, in order to understand both the indexing in the library and the directions of turn of every motor.

#### Motors

This arm has six Dynamixel servo motors to control de position and orientation of the arm. The Dynamixel motors require an id to work. When this arm is acquired, it is important to set the id of each of the motors before start. If this isn't done yet, take a look at the [getting started guide](https://learn.trossenrobotics.com/interbotix/robot-arms/widowx-arm). Starting from the base of the robot arm and going to the gripper, the motors are numbered from 0 to 5. **This indexing is done for coding purposes and it is different from the id given to each motor**. In the library, when the **idx** is required, it refers to the defined indexing where the first motor is 0 and the last motor&mdash;the one that opens and closes the gripper&mdash;is 5. A deeper explanation can be found in the first comments of the [WidowX.h](https://github.com/LeninSG21/WidowX/blob/master/WidowX/WidowX.h) file.

The motors in the Robot Arm are the following:

- [Dynamixel MX-28](http://emanual.robotis.com/docs/en/dxl/mx/mx-28/): Motors idx 0 and 3
- [Dynamixel MX-64](http://emanual.robotis.com/docs/en/dxl/mx/mx-64/): Motors idx 1 and 2
- [Dynamixel AX-12a](http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/): Motors idx 4 and 5

#### Coordinate Systems

To analyze the WidowX, eight coordinate systems are proposed: the base system, the origin, the systems 1-5 for each articular value and the tool system. The sixth motor controls the grip and it does not affect the rotation or the position of the arm, which is why it is not considered an articular value. The following image shows the arm in its centered position as well as the eight coordinate systems proposed.

![Image of WidowX ISO](https://github.com/LeninSG21/WidowX/blob/master/widowISO.JPG)

It is seen that the systems {0}, {1} and {2} share the same origin, while {4} and {5} share theirs. Also, when centered, the tool system&mdash;whose origin is at the center of the gripper&mdash;has the same orientation of the base and origin systems.

The following image shows the eight coordinate systems as well as the articular values with their positive turn and the distances between the origin of the different coordinate systems, expressed as variables.

![Image of WidowX Front](https://github.com/LeninSG21/WidowX/blob/master/widowFront.JPG)

With this definitions of turn and coordinate systems, the center position becomes the real home of the WidowX in this analysis, understanding that the home position is the one in which all the articular values are at 0°.

---

**_NOTE_** Although the center position is the home for the kinematic analysis, it is called **center position**, which is different from the **home position** used in the WidowX.h library. It was chosen like this because the BioloidController has a pose called Home that moves the arm into a different position, not suitable for the analysis.

---

**_TIP_** To understand better the kinematic analysis, take a look to the [WidowX_Kinematics](https://github.com/LeninSG21/WidowX/blob/master/Matlab/WidowX_Kinematics.mlx) Matlab live script. It uses the Peter Corke [Robotics Toolbox](https://la.mathworks.com/matlabcentral/fileexchange/68542-robotics-toolbox-for-matlab), so you need to install it to run the simulations of the arm.

---
