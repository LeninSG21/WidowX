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

![ROS Diagram](https://github.com/LeninSG21/WidowX/blob/master/ROS/InterconnectionROS.png)

In here, we see the different blocks that interconnect to allow the controller to move the WidowX Arm. The controller sends the HID package to the node *ds4_receiver* that is in charge of reading this information and publish the appropriate message to the topic *controller_message*. The node *controller_msg_listener* is subscribed to said topic, and when it receives a message, the node arranges the message into the [data format](https://github.com/LeninSG21/WidowX/tree/master/Arduino%20Library/Examples#data-format) that will be sent to the ArbotiX that is running the **MoveWithController.ino** code. 

Notice also that each node is being executed by different computers. This is ideal to run move the WidowX remotely. However, it is not necessary to run it separately. You can execute this nodes in the same computer. If you want to execute it with different PCs connected via LAN, check the section [Remote Connection](https://github.com/LeninSG21/WidowX/blob/master/ROS/README.md#remote-connection) section.

## Message structure

The *controller_msg_listener* expects to receive a string with six numbers separated by comas. Each of these numbers resembles the [data format](https://github.com/LeninSG21/WidowX/tree/master/Arduino%20Library/Examples#data-format) specified for the [MoveWithController.ino](https://github.com/LeninSG21/WidowX/blob/master/Arduino%20Library/Examples/MoveWithController/MoveWithController.ino) code. So each number is just a byte. Therofe, you could use any publisher you want, from any controller you desire. The subscriber and the microcontroller will work just fine, again, as long as you send the message with the appropriate data format.

## Installation

The code requires Python 2.7 to run. Since it is included in Ubuntu by default, you shouldn't need to install anything. However, if for some misterious reason you do not have Python, make sure to install it.

The obvious first step is to install ROS in your computer. To do that, follow the steps in the [ROS Kinetic installation](http://wiki.ros.org/kinetic/Installation) page.

Once you've donde that, download this repo and copy the **ds4_2_widow** package into the **catkin_ws** directory
```sh
$ cp -r ds4_2_widow ~/catkin_ws/src
```

Then, build the package with the instruction **catkin_make**. Sometimes, it requires to erase the build and devel folders. If so, run the first line of the following example.

```sh
$ cd ~/catkin_ws
$ rm -rf build devel
$ catkin_make
$ source devel/setup.bash
```
Finally, before running the ROS nodes, don't forget to start the master with `$ roscore`

## DS4 Receiver

The DS$ controller communicates with the computer following the **Human Interface Device** standard. The data format in which the DS4 sends the actions on the controller is described in the [DS4-USB](https://www.psdevwiki.com/ps4/DS4-USB) page.

> **Note** It describes the HID package when connected with cable, since the package sent when connected via Bluetooth is different

When you connect the DS4 to a USB port in Ubuntu, an hidraw file is created under **/dev**. Before using it with this node, it is important to enable the lecture from it. To do it, first identify the hidraw number assigned to the controller with the following command

```sh
$ ls -l /dev/hidraw*
```

This will show all the HID devices connected. If you do not know which one corresponds to the controller, run the command before plugging the DS4 and after plugging it to determine the hidraw number. Once you've done that, you'll need to enable the lecture with the following commad

```sh
$ sudo chmod a+r /dev/hidrawN
```

Notice that you have to substitute *N* with the actual number of your device. 

Finally, start the node with the command
```sh
$ rosrun ds4_2_widow ds4_receiver
```
A list containing all the hidraw devices found will be shown. Then, you'll be prompted to select the number of the hidraw device and the code will start executing. You will see in the terminal the message that is being sent to the topic. Move the josticks and see how the message change.

### DS4 Controller Mapping

## Controller Message Receiver

This node is in charge of reading the message string from the topic and parsing it into the data format to be sent via serial to the ArbotiX. For it to work, you need to connect the ArbotiX to a USB port and allow the lecture and writing of the port. To do that, you have to list the USB ports to determine which one corresponds to your ArbotiX

```sh
$ ls -l /dev/ttyUSB*
```
This will show all the ttyUSB devices connected. If you do not know which one corresponds to the ArbotiX, run the command before plugging the microcontroller and after plugging it to determine the ttyUSB number. Once you've done that, you'll need to enable the lecture and writing with the following commad

```sh
$ sudo chmod a+rw /dev/ttyUSBN
```
Notice that you have to substitute *N* with the actual number of your device.

Before running the node, be sure to turn have the ArbotiX connected to the appropriate power source. Remember that for the arm to work, the jumper in the ArbotiX must be set to Vin to indicate that it will be powered through an external power source. If you leave it in USB and leave the arm connected, it will consume more current to move the arm and it will potentially kill your computer. Run the following command once you've done that
```sh
$ rosrun ds4_2_widow controller_msg_receiver
```
A list containing all the ttyUSB devices found will be shown. Then, you'll be prompted to select the number of the ttyUSB device and the code will start executing. 

The first step is to establish the communication with the ArbotiX. Hence, the code first opens the serial port at 115,200 bps. You'll receive some useful messages, one of them saying that the WidowX is starting and the next messages indicating the voltage level. This sequence belongs to the WidowX.h class and checks that the voltage is above 10V to operate appropriately. After that, the code does a handshake with the **MoveWithController.ino** code, where it receives from the ArbotiX **ok\n**. Then, it has to send **ok\n** and finally wait for a last **ok\n** from the microcontroller that indicates that it is ready to receive the 6 bytes messages to control the WidowX. 

The last step of the initialization is asking the user to press the PS button to start the lecture of the DS4. This button is handled by the **ds4_receiver**. The **controller_msg_receiver** waits until the string **"start"** is received. Once this happens, it beggins the normal operation. 

During the normal loop, this node will print into the terminal the message that is being received. Move the controller to see how this message changes. Also, the robot arm should be moving by now. If it moves as expected, play a little and get used to the controls!

## Remote Connection
Hello there!


