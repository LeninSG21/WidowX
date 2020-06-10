# ROS Package for WidowX
If you are working in Ubuntu and you need to control your WidowX Robot Arm with a controller, either directly or with a remote desktop,
this ROS package might be exactly what you need.

Firs of all, if you do not know what ROS is, take a look to its [official website](https://www.ros.org/) to understand it and to see if it
suits your project. Once you've done that, continue reading :D

> **NOTE** The package was developed in Ubuntu 16.04 LTS with ROS Kinetic Kame

In this directory you will find a folder named **ds4_w_widow**. Now, why ds4? DS4 stands for Dual Shock 4, and it is the controller the 
Play Station 4 uses. By now, you might have guessed that this package was developed to control the WidowX Arm with a DS4 in mind. Also,
you might think *"oh crap, I do not have a DS4, this is useless"*. But wait a minute, you can actually take advantage of this package and use
it, since the node dedicated to connect with the ArbotiX of the WidowX arm is universal (as long as you receive the correct message through 
the topic) and the code for the ArbotiX ([MoveWithController.ino](https://github.com/LeninSG21/WidowX/blob/master/Arduino%20Library/Examples/MoveWithController/MoveWithController.ino)
is also universal. So lets see how it works and so that you can adapt it to your own controller.

First, lets take a look to the following diagram.

![ROS Diagram](https://github.com/LeninSG21/WidowX/blob/master/ROS/ROS-Diagram.png)

In here, we see the different blocks that interconnect to allow the controller to move the WidowX Arm. The controller sends the HID package to the node that is in charge of reading this information and publish the appropriate information to the 
