# WidowX

Includes an Arduino library based on [Bioloid.h](https://github.com/froody/bioloid/blob/master/firmware/motion/bioloid.h) and [ax12.h](https://github.com/vanadiumlabs/arbotix/blob/master/libraries/Bioloid/ax12.h) to simplify the control of the [WidowX Robot Arm](https://www.trossenrobotics.com/widowxrobotarm) of [Trossen Robotics](https://www.trossenrobotics.com/). It also contains the analysis of the kinematics of this robot in a Matlab live script.

Inside some of its functions it uses the Matrix object as defined in [BasicLinearAlgebra.h](https://github.com/tomstewart89/BasicLinearAlgebra). Hence, it requires installation in order to work properly.

## Installation

Download the WidowX folder and copy it into the Arduino/libraries folder. It should appear in the **include library** section.
