# Arduino Library Examples

In this directory you will find some useful codes to implement with your WidowX Robot Arm right away. Take a look to how they work and modify them to suit your needs!

## Test Arm

The `testArm.ino` file is ideal to check if the installation was done properly. Just load it into your ArbotiX and watch the arm move from rest, to home, to center and back to rest. Also, check the Serial Monitor to observe some useful messages, like the operation voltage of the arm.

## How to Use

The `HowToUse.ino` file allows you to interact with some of the functions of the WidowX library through serial commands. You can send the arm to the preset positions (Home, Center and Rest) or move each servo individually either by giving the desired angle (as seen from the coordinate system described for this library, where the center position of each servo is equal to 0°) or by giving the position, as described in the appropriate dynamixel datasheet (from 0-4095 for MX-28 and MX-64 and from 0-1023 fro the AX-12a). 

Also, you can see the effect of relaxing the servos and activating the torque. Make sure to grab the arm or to have it in rest position before calling the relax command. Finally, you can watch a demonstration of the different inverse kinematics functions. Look at the serial terminal to read the inverse kinematics function that is being used and the point and rotation that is desired to achieve. Then look the arm do it. This is a good way to understand the different effects each function has and by looking at the code you can learn how to implement it.

This code depends on the BasicLinearAlgebra.h library, so be sure to download for the code to run appropriately.

## Move With Controller

The `MoveWithController.ino` file is designed to receive a message via the serial port to move the WidowX arm with a controller. This code only interprets the message received and sends the appropriate information to the WidowX library to move the arm. It does not care who sends the message and how it build it. Therefore, you can use this code with any controller and button mapping you want, as long as you follow the message structure defined next.

### Data format
<table>
  <thead>
    <tr>
      <th>Byte Index</th>
      <th>bit 7</th>
      <th>bit 6</th>
      <th>bit 5</th>
      <th>bit 4</th>
      <th>bit 3</th>
      <th>bit 2</th>
      <th>bit 1</th>
      <th>bit 0</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>0</td>
      <td>sign</td>
      <td colspan = 7>speed in x</td>
    </tr>
    <tr>
      <td>1</td>
      <td>sign </td>
      <td colspan = 7>speed in y</td>
    </tr>
    <tr>
      <td>2</td>
      <td>sign </td>
      <td colspan = 7>speed in z</td>
    </tr>
    <tr>
      <td>3</td>
      <td colspan = 8>speed for Gamma</td>
    </tr>
    <tr>
      <td>4</td>
      <td colspan = 8>speed for Q5</td>
    </tr>
    <tr>
      <td>5</td>
      <td>Sign of Gamma speed</td>
      <td>Sign of Q5 speed </td>
      <td colspan = 2>
        <p> Grip movement </p>
        <table>
          <thead>
            <tr>
              <th>2</th>
              <th>1</th>
              <th >other</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>close grip</td>
              <td>open grip</td>
              <td>void</td>
            </tr>
          </tbody>
         </table>
      </td>
      <td colspan = 4>
        <p>Options</p>
        <table>
          <thead>
            <tr>
              <th>other</th>
              <th>7</th>
              <th>6</th>
              <th>5</th>
              <th>4</th>
              <th>3</th>
              <th>2</th>
              <th>1</th>
              <th>0</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>void</td>
              <td>USER_FRIENDLY</td>
              <td>POINT_MOVEMENT</td>
              <td>torque</td>
              <td>relax</td>
              <td>center</td>
              <td>home</td>
              <td>rest</td>
              <td>Process movement</td>
            </tr>
          </tbody>
         </table>
      </td>
    </tr>
  </tbody>  
</table>

For the speed values in x, y and z, the range goes from 0 to 127 (7 bits). To determine the direction, the sign bit is provided where **0** means the velocity is **positive** and **1** that it is **negative**. Therefore, 0 is equivalent to 128, since the speed bits are effectively the same; 127 is max positive speed and 255 would be max negative speed. In the cases of the Gamma and Q5 speed, the value goes from 0 to 255 (8 bits), and the direction bit is assigned in the options nibble. 

First, the code receives the 6 bytes and saves them into a buffer, like so
```cpp
Serial.readBytes(buff, 6);
```
Then, it extracts each piece of information from its corresponding bits. For example, to get the velocity in y as a signed integer, the following is done
```cpp
int vy = buff[1] & 0x7F; //Obtain the speed from the 7 bits
if(buff[1]>>7) //Check if the bit sign is active. If it is 1, then vy should be negative
  vy = -vy;
```
To do the same with the velocity of Q5, the code is as follows
```cpp
int vq5 = buff[4];
if ((buff[5] >> 6) & 1) //Obtain the sign bit from the LSB
  vq5 = -vq5;
```
The options nibble can have up to 15 different options (0 must be left untouched), but only 7 are currently being used. That means that you could add even more functionalities with the exact same code. You would just need to add more cases in the switch statement that handles options. However, leave the 0 as the flag that indicates that no option was given. Since these options have higher priority than the movement of the arm through joysticks, if the code detects that the option nibble is different from 0, it will **not** process the other bytes of the message and will not move the arm according to the joysticks.

```cpp
byte options = buff[5] & 0xF; //Get options bits
if (options == 0) //No given option (higher priority)
{
  //Read velocities and move the arm accordingly
}
else{
  switch(options)
  {
    case 1..7:
      //corresponding action
    default:
      break;
  }
}
```

Also, in the grip movement bits, two more actions could be added, apart from open and close. That means that with this 6 bytes message configuration, up to 10 more actions can be included with the same code. You just need to add more cases in the corresponding area.

### Move Options
There are two movement options with this code: the **USER_FRIENDLY** and the **POINT_MOVEMENT** options. By default, the program initializes with the USER_FRIENDLY mode active, but you can change to POINT_MOVEMENT mode (and viceversa) with the options nibble. 
While the message received is the same, the user experience varies depending on the selected mode. 

#### USER_FRIENDLY
As the name suggests, the easiest way to control the arm with a controller is with the USER_FRIENDLY mode. With this mode, the velocity in x makes the arm go forward or backward from the system {1}. That is, if you want it to go to the front, you just send a positive vx, and if you want it to go back, a negative vx. With the velocity in y, you can make the arm turn counter clockwise with a positive vy and clockwise with a negative vy (as seen from above). With vz, you make it go up or down. This is ideal if the arm has a camera on it and it moves along with the arm. Then, the controlling experience is the same no matter the orientation of the arm.

#### POINT_MOVEMENT
The POINT_MOVEMENT mode is easier to understand by a computer. With this mode, instead of moving the arm itself, you are adjusting the 3D point at the center of the grip according to the base of the robot. So for the vz, it works exactly the same as with the USER_FRIENDLY mode. However, when the changes in vx and vy change the coordinate of the center of the grip. Let's say that the coordinate of the gripper is (20,0,20)cm. If you give the vx a positive value, the arm would go to the front, extending itself, and the new coordinate could be (30,0,20)cm, for example. This is exactly as expected, and the same would happen with the USER_FRIENDLY mode. 

Now, lets say that the coordinate of the gripper as seen from the base of the robot is (0,20,20)cm, and you give the same positive velocity in x. With the USER_FRIENDLY mode, you would expect the arm to do the same as it did before, which is to extend itself, reaching let's say the coordinate (0,30,20)cm. But with the POINT_MOVEMENT mode, what you the program would interpret is that you need to move said point to another x-coordinate while mainting the y value. So instead, you might obtain this new coordinate for the gripper (10,20,20)cm. While this might seem unintuitive for a human controlling the robot, it is the easiest way to interpret it for a computer, especially if you need it to reach a specific coordinate. 

If you are unsure which one to use, play safe and leave it in the USER_FRIENDLY mode. Nonetheless, you can try both and see which one suits you best. For the USER_FRIENDLY mode, the method used is `moveArmWithSpeed(vx,vy,vz,vg,initial_time)`. For the POINT_MOVEMENT mode, the method is `movePointWithSpeed(vx,vy,vz,vg,initial_time)`. It is important to notice that the movement programmed with this code is determined by the inverse kinematics with a gamma given. Take a look a the [Move Arm](https://github.com/LeninSG21/WidowX/tree/master/Arduino%20Library#move-arm) section in the libraries documentation to understand better how these functions work.
