# WidowX
Hello there! ~~General Kenobi!~~ In this GitHub you'll find some useful codes to start playing around with your Trossen Robotics WidowX Robot Arm. In the section [Arduino Library](https://github.com/LeninSG21/WidowX/tree/master/Arduino%20Library), you'll find the WidowX library, which was designed to give make your life easier. It has inside of it some functions that will allow you to set each motor individually, to move the arm to a given point with an specific rotation in the time you desire, and to even control the arm's movement with a controller! Also, some example codes have been provided, to test the installation of the libraries required and the movement of the arm, to try some functions and understand how they work, and a code ready to be interfaced with a controller with the Serial port. 

In the [ROS](https://github.com/LeninSG21/WidowX/tree/master/ROS) section, you'll find a package develope to control the WidowX arm with a controller&mdash;more precisely a PS4 Dual Shock 4&mdash;. But if you look at the thouroughly explained documentation, you'll find that you could use any controller you want with the same nodes and Arduino code that I've provided, since you would only need to map the same information into the controller of your preference. 

Since I cannot explain everything once again in here, why don't you check out the specific documentation for each section? I tried to give as much detail as possible, so that the integration of the libraries to your code is as easy as it can be. 

> **TIP** First, read the section below to understand the coordinate system proposed for this robot. It is important since the library was built around this analysis. You don't have to be an expert, you just need to understand the direction of turn of each motor and the 0° position. *Spoiler alert: it is at the center position of each motor*. Once you understand it, go into the Arduino Library section and leave the ROS package at last.

## Understanding the WidowX

In this section the kinematics of the WidowX Rotbot Arm will be described, in order to understand the directions of turn of every motor and the position of each coordinate system.

### Motors

This arm has six Dynamixel servo motors to control de position and orientation of the arm. The Dynamixel motors require an id to work. When this arm is acquired, it is important to set the id of each of the motors before start. If this isn't done yet, take a look at the [getting started guide](https://learn.trossenrobotics.com/interbotix/robot-arms/widowx-arm). Starting from the base of the robot arm and going to the gripper, the motors are numbered from 0 to 5. **This indexing is done for coding purposes and it is different from the id given to each motor**. In the library, when the **idx** is required, it refers to the defined indexing where the first motor is 0 and the last motor&mdash;the one that opens and closes the gripper&mdash;is 5. A deeper explanation can be found in the [id vs idx](https://github.com/LeninSG21/WidowX/tree/master/Arduino%20Library#id-vs-idx) section of the Arduino Library documentation.

The motors in the Robot Arm are the following:

- [Dynamixel MX-28](http://emanual.robotis.com/docs/en/dxl/mx/mx-28/): Motors idx 0 and 3
- [Dynamixel MX-64](http://emanual.robotis.com/docs/en/dxl/mx/mx-64/): Motors idx 1 and 2
- [Dynamixel AX-12a](http://emanual.robotis.com/docs/en/dxl/ax/ax-12a/): Motors idx 4 and 5

When you hear about the *positon of the motor*, know that I'm referring to the numeric value that sets the angle of the rotor. In the datasheets od the motors, it is called the [Goal Position](https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#goal-position-30). this goal position goes from 0 to 4095 for the MX-28 and MX-64 and from 0 to 1023 for the AX-12a

### Coordinate Systems

To analyze the WidowX, eight coordinate systems are proposed: the base system, the origin, the systems 1-5 for each articular value and the tool system. The sixth motor controls the grip and it does not affect the rotation or the position of the arm, which is why it is not considered an articular value. The following image shows the arm in its centered position as well as the eight coordinate systems proposed.

![Image of WidowX ISO](https://github.com/LeninSG21/WidowX/blob/master/widowISO.JPG)

It is seen that the systems {0}, {1} and {2} share the same origin, while {4} and {5} share theirs. Also, when centered, the tool system&mdash;whose origin is at the center of the gripper&mdash;has the same orientation of the base and origin systems.

The following image shows the eight coordinate systems as well as the articular values with their positive turn and the distances between the origin of the different coordinate systems, expressed as variables.

![Image of WidowX Front](https://github.com/LeninSG21/WidowX/blob/master/widowFront.JPG)

With this definitions of turn and coordinate systems, the center position becomes the real home of the WidowX in this analysis, understanding that the home position is the one in which all the articular values are at 0°.

---

**NOTE** Although the center position is the home for the kinematic analysis, it is called **center position**, which is different from the **home position** used in the WidowX.h library. It was chosen like this because the BioloidController has a pose called Home that moves the arm into a different position, not suitable for the analysis.

---

**TIP** To understand better the kinematic analysis, take a look to the [WidowX_Kinematics](https://github.com/LeninSG21/WidowX/blob/master/Matlab/WidowX_Kinematics.mlx) Matlab live script. It uses the Peter Corke [Robotics Toolbox](https://la.mathworks.com/matlabcentral/fileexchange/68542-robotics-toolbox-for-matlab), so you need to install it to run the simulations of the arm.

---
