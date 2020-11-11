/*
WidowX.cpp - Library to control WidowX Robotic Arm of Trossen Robotics in Arduino 1.0.6
Created by Lenin Silva, November, 2020
 
 MIT License

Copyright (c) 2020 LeninSG21

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include "WidowX.h"
#include "math.h"
#include "poses.h"
#include "ax12.h"

//////////////////////////////////////////////////////////////////////////////////////
/*
    *** PUBLIC FUNCTIONS ***
*/

//INITIALIAZERS
/*
 * Constructor of the class. Needs to be called first to create an instance. 
 * It assigns values to some constants and fills the id array from 1 to 6. 
*/
WidowX::WidowX()
    : SERVOCOUNT(6), L0(9), L1(14), L2(5), L3(14),
      L4(14), D(sqrt(pow(L1, 2) + pow(L2, 2))), alpha(atan2(L1, L2)), sa(sin(alpha)),
      ca(cos(alpha)), isRelaxed(0), DEFAULT_TIME(2000)
{

    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        id[i] = i + 1;
    }
}

/*
 * This is NOT a required function in order to work properly. It sends the arm 
 * to rest position. It also checks the voltage with the function checkVoltage(). 
 * If relax != 0, then the torque is disabled after it reaches the rest position.
*/
void WidowX::init(uint8_t relax)
{
    delay(10);
    checkVoltage();
    // moveRest();
    // delay(100);
    // if (relax)
    //     relaxServos();
}

/*
 * Moves to the arm to the rest position, which is when the arm is resting over itself. 
 * Reads the pose from PROGMEM. Updates point once it's done
 * 
*/
// void WidowX::moveRest()
// {
//     getCurrentPosition();
//     interpolateFromPose(Rest, DEFAULT_TIME);
//     updatePoint();
// }

//Get Information
/* 
 * Checks that the voltage values are adequate for the robotic arm. If it is below
 * 10V, it remains in a loop until the voltage increases. This is to prevent
 * damage to the arm. Also, it sends through the serial port of the ArbotiX some 
 * information about the voltage, which can be then seen by the serial monitor or
 *  received with another interface connected to the ArbotiX serial
*/
void WidowX::checkVoltage()
{
    // wait, then check the voltage (LiPO safety)
    float voltage = (ax12GetRegister(1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    Serial.println("###########################");
    Serial.print("System Voltage: ");
    Serial.print(voltage);
    Serial.println(" volts.");
    while (voltage <= 10.0)
    {
        Serial.println("Voltage levels below 10v, please charge battery.");
        delay(1000);
        voltage = (ax12GetRegister(1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    }
    if (voltage > 10.0)
    {
        Serial.println("Voltage levels nominal.");
    }
    Serial.println("###########################");
}