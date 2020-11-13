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
    *** GLOBAL VARIABLES ***
*/
float phi2;
float a, b, c, cond;
float q1, q2, q3, q4, q5, q6;
int posQ4, posQ5, posQ6;
const float limPi_2 = 181 * M_PI / 360;
const float lim5Pi_6 = 5 * M_PI / 6;
const float q2Lim[] = {-limPi_2, limPi_2};
const float q3Lim[] = {-limPi_2, lim5Pi_6};
const float q4Lim[] = {-11 * M_PI / 18, limPi_2};
long t0;
int currentTime, remainingTime;
float curr_2, curr_3;

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
    moveRest();
    // delay(100);
    // if (relax)
    //     relaxServos();
}

//Preloaded Poses
/*
 * Moves to the arm to the center of all motors, forming an upside L. As seen from 
 * the kinematic analysis done for this library, it would be equal to setting all
 * articular values to 0°.  Reads the pose from PROGMEM. Updates point once it's done
*/
void WidowX::moveCenter()
{
    getCurrentPosition();
    interpolateFromPose(Center, DEFAULT_TIME);
    // updatePoint();
}
/*
 * Moves to the arm to the home position as defined by the bioloid controller. 
 * Reads the pose from PROGMEM. Updates point once it's done
*/
void WidowX::moveHome()
{

    getCurrentPosition();
    interpolateFromPose(Home, DEFAULT_TIME);
    // updatePoint();
}

/*
 * Moves to the arm to the rest position, which is when the arm is resting over itself. 
 * Reads the pose from PROGMEM. Updates point once it's done
 * 
*/
void WidowX::moveRest()
{
    getCurrentPosition();
    interpolateFromPose(Rest, DEFAULT_TIME);
    // updatePoint();
}

void WidowX::moveToPose(const unsigned int *pose)
{
    getCurrentPosition();
    interpolateFromPose(pose, DEFAULT_TIME);
    // updatePoint();
}

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

/*
 * This function calls getServoPosition for each of the motors in the arm
*/
void WidowX::getCurrentPosition()
{
    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        getServoPosition(i);
    }
}

int WidowX::getServoPosition(int idx)
{
    uint8_t i = 0;
    uint16_t prev = current_position[idx];
    current_position[idx] = GetPosition(id[idx]);
    while (current_position[idx] == -1)
    {
        i = (i % 10) + 1;
        delay(5 * i);
        current_position[idx] = GetPosition(id[idx]);
    }
    // current_angle[idx] = positionToAngle(idx, current_position[idx]);
    // float_position[idx] = current_position[idx];
    return current_position[idx];
}

//////////////////////////////////////////////////////////////////////////////////////
/*
    *** PRIVATE FUNCTIONS ***
*/

//Poses and interpolation

void WidowX::cubeInterpolation(float q0, float qf, float *w, int time)
{
    const float tf_2_3 = 2 / pow(time, 3);
    const float tf_3_2 = 3 / pow(time, 2);

    w[0] = q0;
    w[1] = 0;
    w[2] = tf_3_2 * (qf - q0);
    w[3] = tf_2_3 * (q0 - qf);

    return;
}

void WidowX::interpolateFromPose(const unsigned int *pose, int remainingTime)
{
    uint8_t i = 0;
    for (i = 0; i < SERVOCOUNT - 1; i++)
    {
        desired_position[i] = pgm_read_word_near(pose + i);
        cubeInterpolation(current_position[i], desired_position[i], W[i], remainingTime);
    }
    t0 = millis();
    currentTime = millis() - t0;
    while (currentTime < remainingTime)
    {
        curr_2 = pow(currentTime, 2);
        curr_3 = pow(currentTime, 3);
        for (i = 0; i < SERVOCOUNT - 1; i++)
        {
            next_position[i] = round(W[i][0] + W[i][1] * currentTime + W[i][2] * curr_2 + W[i][3] * curr_3);
            SetPosition(id[i], next_position[i]);
        }

        delay(10);
        currentTime = millis() - t0;
    }

    SetPosition(id[0], desired_position[0]);
    SetPosition(id[1], desired_position[1]);
    SetPosition(id[2], desired_position[2]);
    SetPosition(id[3], desired_position[3]);
    SetPosition(id[4], desired_position[4]);
    // SetPosition(id[5], desired_position[5]);
    delay(3);
}