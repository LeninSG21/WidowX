/*
WidowX.cpp - Library to control WidowX Robotic Arm of Trossen Robotics
Created by Lenin Silva, June, 2020
 
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
#include "Arduino.h"
#include "WidowX.h"
#include <ax12.h>
#include "math.h"
#include "poses.h"

// #define M_PI 3.14159265358979323846264338327950288   /* pi             */
//#define M_PI_2 1.57079632679489661923132169163975144 /* pi/2           */

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
      ca(cos(alpha)), isRelaxed(0), DEFAULT_TIME(2000), xy_lim(43.0), z_lim_up(52.0), z_lim_down(-26.0),
      gamma_lim(M_PI_2), Kp(60.0 / 127000), Kg(M_PI_2 / 255000), Ks(1024.0 / 255000)
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
    delay(100);
    if (relax)
        relaxServos();
}

//ID HANDLERS
/*
 * Sets the id of the specified motor by idx
*/
void WidowX::setId(int idx, int newID)
{
    if (idx < 0 || idx >= SERVOCOUNT)
        return;
    id[idx] = newID;
}

/*
 * Gets the id of the specified motor by idx.
*/
int WidowX::getId(int idx)
{
    return id[idx];
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
    //updatePoint();
}
/*
 * Moves to the arm to the home position as defined by the bioloid controller. 
 * Reads the pose from PROGMEM. Updates point once it's done
*/
void WidowX::moveHome()
{

    getCurrentPosition();
    interpolateFromPose(Home, DEFAULT_TIME);
    //updatePoint();
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
    //updatePoint();
}

void WidowX::moveToPose(const unsigned int *pose)
{
    getCurrentPosition();
    interpolateFromPose(pose, DEFAULT_TIME);
    //updatePoint();
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

void WidowX::getCurrentPosition(uint8_t until_idx)
{
    for (uint8_t i = 0; i <= until_idx; i++)
    {
        getServoPosition(i);
    }
}

/*
 * This function calls the GetPosition function from the ax12.h library. However, 
 * it was seen that in some cases the value returned was -1. Hence, it made the 
 * arm to move drastically, which, represents a hazard to those around and the arm 
 * itself. That is why this function checks if the returned value is -1. If it is, 
 * reads the current position of the specified motor until the value is not -1. Also, 
 * for every iteration, the delay between lectures varies from 5 to 45ms, to give time 
 * to the motor to respond appropriately. Due to this check conditions, this function 
 * is preferred over the readPose() function from the BioloidController.h library. 
 * The problem is that it might cause a significant delay if the motors keep returning -1. 
 * Nonetheless, it is better to have this delay than to risk the arm’s integrity. 
*/
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
    current_angle[idx] = positionToAngle(idx, current_position[idx]);
    float_position[idx] = current_position[idx];
    return current_position[idx];
}

/**
 * This function returns the current angle of the specified motor.
 * It uses getServoPosition() to prevent failure from reading -1 in the current
 * position
*/
float WidowX::getServoAngle(int idx)
{
    getServoPosition(idx);
    return current_angle[idx];
}

/*
 * Calls the private function updatePoint to load the current point and copies
 * the values into the provided pointer
 */
// void WidowX::getPoint(float *p)
// {
//     updatePoint();
//     p[0] = point[0];
//     p[1] = point[1];
//     p[2] = point[2];
// }

//Torque
/*
 * This function disables the torque of all the servos and sets the global flag isRelaxed to true.
 * WARNING: when using it be careful of the arm's position; otherwise it can be damaged if it 
 * impacts too hard on the ground or with another object
 */
void WidowX::relaxServos()
{
    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        Relax(id[i]);
        delay(10);
    }
    isRelaxed = 1;
}

/*
 * This function enables the torque of every motor. It does not alter their current positions.
 */
void WidowX::torqueServos()
{
    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        TorqueOn(id[i]);
        delay(10);
    }
    isRelaxed = 0;
}

//Move Servo
/*
 * Moves the specified motor (by its idx) to the desired angle in radians. It is important to understand
 * the directions of turn of every motor as described in the documentation in order to select the
 * appropriate angle
*/
void WidowX::moveServo2Angle(int idx, float angle)
{
    if (idx < 0 || idx >= SERVOCOUNT)
        return;
    int pos = angleToPosition(idx, angle);
    int curr = getServoPosition(idx);
    if (curr < pos)
    {
        while (curr < pos)
        {
            SetPosition(id[idx], ++curr);
            delay(1);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id[idx], --curr);
            delay(1);
        }
    }
}

/**
 * Moves the specified motor (by its idx) to the desired position. It does not validate if the position is in the appropriate
 * ranges, so be careful
*/
void WidowX::moveServo2Position(int idx, int pos)
{
    if (idx < 0 || idx >= SERVOCOUNT)
        return;

    int curr = getServoPosition(idx);
    if (curr < pos)
    {
        while (curr < pos)
        {
            SetPosition(id[idx], ++curr);
            delay(1);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id[idx], --curr);
            delay(1);
        }
    }
}

/**
 * Closes or opens the gripper (Q6 | idx = 5): close = 0 --> open, close = 1 --> close in steps of 10
 * Use it inside a loop with a delay to control the smoothness of the turn. Ideal for
 * movement with control or key that is being sent as long as it is pressed
*/
void WidowX::moveGrip(int close)
{
    posQ6 = getServoPosition(5);
    if (close)
    {
        if (posQ6 > 10)
        {
            posQ6 -= 10;
        }
        else
        {
            posQ6 = 0;
        }
    }
    else
    {
        if (posQ6 > 522)
        {
            posQ6 -= 10;
        }
        else if (posQ6 < 502)
        {
            posQ6 += 10;
        }
        else
        {
            posQ6 = 512;
        }
    }
    SetPosition(id[5], posQ6);
}

/**
 * Sets the specified servo to the given position without a smooth tansition.
 * Designed to be used with a controller.
*/
void WidowX::setServo2Position(int idx, int position)
{
    SetPosition(id[idx], position);
}

void WidowX::moveServoWithSpeed(int idx, int speed, long initial_time)
{

    int tf = millis() - initial_time;
    int lim_up = 1023;
    if (idx < 4) //MX-28 | MX_64
    {
        lim_up = 4095;
    }
    float_position[idx] = max(0, min(lim_up, float_position[idx] + speed * Ks * tf));
    SetPosition(id[idx], round(float_position[idx]));
}

//Move Arm
/*
    This function is designed to work with a joystick or controller. It moves the origin 
    of the grippers coordinate system. To move the arm with speed control, use this 
    function inside a loop. At the beginning of the loop, the initial_time is 
    set—for example, with the function millis() in Arduino. 
    The function receives four different velocities:
    •	vx: velocity to move the x-coordinate as seen from the base of the robot
    •	vy:  velocity to move the y-coordinate as seen from the base of the robot
    •	vz:  velocity to move the z-coordinate as seen from the base of the robot
    •	vg:  velocity to move the angle of the gripper as seen from the base of the robot
    This function was designed considering that speed values range from [-127,127] for 
    vx, vy and vz and [-255,255] for vg. Higher values are mathematically possible, 
    but will make the arm move faster, so be cautious. This function offers a 
    better controlling experience for machines.
*/
void WidowX::movePointWithSpeed(int vx, int vy, int vz, int vg, long initial_time)
{

    int tf = millis() - initial_time;
    speed_points[0] = max(-xy_lim, min(xy_lim, speed_points[0] + vx * Kp * tf));
    speed_points[1] = max(-xy_lim, min(xy_lim, speed_points[1] + vy * Kp * tf));
    speed_points[2] = max(z_lim_down, min(z_lim_up, speed_points[2] + vz * Kp * tf));
    global_gamma = max(-gamma_lim, min(gamma_lim, global_gamma + vg * Kg * tf));
    setArmGamma(speed_points[0], speed_points[1], speed_points[2], global_gamma);
}

/*
    This function also moves the arm when controlled with a joystick or controller, 
    just as movePointWithSpeed(). However, while the other function moves according 
    to the desired translation and rotation of the origin of the grippers coordinate 
    system, this function moves the arm to the front or back with vx, it turns it 
    around with vy and with vy it goes up and down. This function was designed 
    considering that speed values range from [-127,127] for vx, vy and vz and 
    [-255,255] for vg. Higher values are mathematically possible, but will make 
    the arm move faster, so be cautious. This function offers a better controlling 
    experience for human operators.
*/
void WidowX::moveArmWithSpeed(int vx, int vy, int vz, int vg, long initial_time)
{
    int tf = millis() - initial_time;

    float theta_0 = atan2(speed_points[1], speed_points[0]);
    float delta_theta = 4 * vy * Kg * tf;

    float magnitude_U0 = sqrt(pow(speed_points[0], 2) + pow(speed_points[1], 2));
    float deltaU = vx * Kp * tf;

    float magnitude_Uf = magnitude_U0 + deltaU;
    float theta_f = theta_0 + delta_theta;
    speed_points[0] = magnitude_Uf * cos(theta_f);
    speed_points[1] = magnitude_Uf * sin(theta_f);
    speed_points[2] = max(z_lim_down, min(z_lim_up, speed_points[2] + vz * Kp * tf));
    global_gamma = max(-gamma_lim, min(gamma_lim, global_gamma + vg * Kg * tf));
    setArmGamma(speed_points[0], speed_points[1], speed_points[2], global_gamma);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot.
 * It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function
 * only affects Q1, Q2 and Q3. It interpolates the step using a cubic interpolation with the default time.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmQ4(float Px, float Py, float Pz)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();

    // Serial.println("CURRENT");
    // Serial.print("q1: ");
    // Serial.println(current_angle[0]);
    // Serial.print("q2: ");
    // Serial.println(current_angle[1]);
    // Serial.print("q3: ");
    // Serial.println(current_angle[2]);
    // Serial.print("q4: ");
    // Serial.println(current_angle[3]);
    // Serial.print("q5: ");
    // Serial.println(current_angle[4]);
    // Serial.print("q6: ");
    // Serial.println(current_angle[5]);

    if (getIK_Q4(Px, Py, Pz))
    {
        Serial.println("No solution for IK!");
        return;
    }
    // Serial.println("\nDesired");
    // Serial.print("q1: ");
    // Serial.println(desired_angle[0]);
    // Serial.print("q2: ");
    // Serial.println(desired_angle[1]);
    // Serial.print("q3: ");
    // Serial.println(desired_angle[2]);
    // Serial.print("q4: ");
    // Serial.println(desired_angle[3]);
    // Serial.print("q5: ");
    // Serial.println(desired_angle[4]);
    // Serial.print("q6: ");
    // Serial.println(desired_angle[5]);

    // return;

    interpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot.
 * It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function
 * only affects Q1, Q2 and Q3. It interpolates the step using a cubic interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmQ4(float Px, float Py, float Pz, int time)
{
    if (isRelaxed)
        torqueServos();

    t0 = millis();
    getCurrentPosition();
    if (getIK_Q4(Px, Py, Pz))
    {
        Serial.println("No solution for IK!");
        return;
    }

    remainingTime = time - (millis() - t0);
    interpolate(remainingTime);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
 * desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
 * to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
 * It interpolates the step using a cubic interpolation with the default time.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmGamma(float Px, float Py, float Pz, float gamma)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Gamma(Px, Py, Pz, gamma))
    {
        Serial.println("No solution for IK!");
        return;
    }

    interpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
 * desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
 * to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
 * It interpolates the step using a cubic interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmGamma(float Px, float Py, float Pz, float gamma, int time)
{
    if (isRelaxed)
        torqueServos();
    t0 = millis();
    getCurrentPosition();
    if (getIK_Gamma(Px, Py, Pz, gamma))
    {
        Serial.println("No solution for IK!");
        return;
    }

    remainingTime = time - (millis() - t0);
    interpolate(remainingTime);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matrix of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using a cubic interpolation with the default time.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
 * Rd is a Matrix object as defined by BasicLinearAlgebra.h
*/
void WidowX::moveArmRd(float Px, float Py, float Pz, float Rd[3][3])
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Rd(Px, Py, Pz, Rd))
    {
        Serial.println("No solution for IK!");
        return;
    }
    interpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matrix of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using a cubic interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
 * Rd is a Matrix object as defined by BasicLinearAlgebra.h
*/
void WidowX::moveArmRd(float Px, float Py, float Pz, float Rd[3][3], int time)
{
    if (isRelaxed)
        torqueServos();
    t0 = millis();
    getCurrentPosition();
    if (getIK_Rd(Px, Py, Pz, Rd))
    {
        Serial.println("No solution for IK!");
        return;
    }
    remainingTime = time - (millis() - t0);
    interpolate(remainingTime);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, 
 * and with the desired rotation of the coordinate system of the gripper, as 
 * seen from the base of the robot. It uses getIK_RdBase. This function affects 
 * Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation 
 * with the default time. If there is no solution for the IK, the arm does not move, 
 * and a message is printed into the serial monitor.
*/
void WidowX::moveArmRdBase(float Px, float Py, float Pz, float RdBase[3][3])
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_RdBase(Px, Py, Pz, RdBase))
    {
        Serial.println("No solution for IK!");
        return;
    }

    interpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, 
 * and with the desired rotation of the coordinate system of the gripper, as 
 * seen from the base of the robot. It uses getIK_RdBase. This function affects 
 * Q1, Q2, Q3, Q4, and Q5. It interpolates the step using a cubic interpolation 
 * with the given time in milliseconds. If there is no solution for the IK, the arm does not move, 
 * and a message is printed into the serial monitor.
*/
void WidowX::moveArmRdBase(float Px, float Py, float Pz, float RdBase[3][3], int time)
{
    if (isRelaxed)
        torqueServos();
    t0 = millis();
    getCurrentPosition();
    if (getIK_RdBase(Px, Py, Pz, RdBase))
    {
        Serial.println("No solution for IK!");
        return;
    }

    remainingTime = time - (millis() - t0);
    interpolate(remainingTime);
}

//Sequence

void WidowX::performSequenceGamma(float **seq, int num_poses)
{
    for (int i = 0; i < num_poses; i++)
    {
        moveArmGamma(seq[i][0], seq[i][1], seq[i][2], seq[i][3], (int)seq[i][4]);
    }
}

//Rotations
void WidowX::rotz(float angle, float Rz[3][3])
{
    Rz[0][0] = cos(angle);
    Rz[0][1] = -sin(angle);
    Rz[0][2] = 0;

    Rz[1][0] = sin(angle);
    Rz[1][1] = cos(angle);
    Rz[1][2] = 0;

    Rz[2][0] = 0;
    Rz[2][1] = 0;
    Rz[2][2] = 1;

    // Rz = {cos(angle), -sin(angle), 0,
    //       sin(angle), cos(angle), 0,
    //       0, 0, 1};
}

void WidowX::roty(float angle, float Ry[3][3])
{
    Ry[0][0] = cos(angle);
    Ry[0][1] = 0;
    Ry[0][2] = sin(angle);

    Ry[1][0] = 0;
    Ry[1][1] = 1;
    Ry[1][2] = 0;

    Ry[2][0] = -sin(angle);
    Ry[2][1] = 0;
    Ry[2][2] = cos(angle);

    // Ry = {cos(angle), 0, sin(angle),
    //       0, 1, 0,
    //       -sin(angle), 0, cos(angle)};
}

void WidowX::rotx(float angle, float Rx[3][3])
{
    Rx[0][0] = 1;
    Rx[0][1] = 0;
    Rx[0][2] = 0;

    Rx[1][0] = 0;
    Rx[1][1] = cos(angle);
    Rx[1][2] = -sin(angle);

    Rx[2][0] = 0;
    Rx[2][1] = sin(angle);
    Rx[2][2] = cos(angle);

    // Rx = {1, 0, 0,
    //       0, cos(angle), -sin(angle),
    //       0, sin(angle), cos(angle)};
}

//////////////////////////////////////////////////////////////////////////////////////
/*
    *** PRIVATE FUNCTIONS ***
*/

//Conversions
float WidowX::positionToAngle(int idx, int position)
{
    if (idx == 0 || idx == 3 || idx == 2) //MX-28 || MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        return 0.00153435538637 * (position - 2047.5);
    else if (idx == 1) //MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        return -0.00153435538637 * (position - 2047.5);
    else //AX-12
        //0-1023
        //0° - 300°
        return 0.00511826979472 * (position - 511.5);
}

int WidowX::angleToPosition(int idx, float angle)
{
    if (abs(angle) > M_PI)
    {
        if (angle > 0)
            angle -= 2 * M_PI;
        else
            angle += 2 * M_PI;
    }
    if (idx == 0 || idx == 3 || idx == 2) //MX-28 || MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        // return (int)651.74 * angle + 2048;
        return round(651.739492 * angle + 2047.5);

    else if (idx == 1) //MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        return round(-651.739492 * angle + 2047.5);
    else //AX-12
        //0-1023
        //0° - 300°
        return round(195.378524405 * angle + 511.5);
}

//Poses and interpolation

// void WidowX::updatePoint()
// {
//     getCurrentPosition();
//     q1 = current_angle[0];
//     q2 = current_angle[1];
//     q3 = current_angle[2];
//     q4 = current_angle[3];

//     phi2 = D * cos(alpha + q2) + L3 * cos(q2 + q3) + L4 * cos(q2 + q3 + q4);
//     global_gamma = -q2 - q3 - q4;
//     point[0] = cos(q1) * phi2;
//     point[1] = sin(q1) * phi2;
//     point[2] = L0 + D * sin(alpha + q2) + L3 * sin(q2 + q3) + L4 * sin(q2 + q3 + q4);
//     // speed_points[0] = point[0];
//     // speed_points[1] = point[1];
//     // speed_points[2] = point[2];
// }

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

void WidowX::interpolate(int remTime)
{
    uint8_t i;
    for (i = 0; i < SERVOCOUNT - 1; i++)
    {
        desired_position[i] = angleToPosition(i, desired_angle[i]);
        cubeInterpolation(current_position[i], desired_position[i], W[i], remTime);
    }

    t0 = millis();
    currentTime = millis() - t0;
    while (currentTime < remTime)
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
    delay(3);
}

void WidowX::interpolateFromPose(const unsigned int *pose, int remTime)
{
    uint8_t i;
    for (i = 0; i < SERVOCOUNT - 1; i++)
    {
        desired_position[i] = pgm_read_word_near(pose + i);
        cubeInterpolation(current_position[i], desired_position[i], W[i], remTime);
    }

    t0 = millis();
    currentTime = millis() - t0;
    while (currentTime < remTime)
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

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
 * desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
 * to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
 * It does not interpolate the step, since it is designed to be used by a controller that will move the arm
 * smoothly. If there is no solution for the IK, the arm does not move.
*/
void WidowX::setArmGamma(float Px, float Py, float Pz, float gamma)
{
    if (isRelaxed)
        torqueServos();

    //getCurrentPosition();
    if (getIK_Gamma_Controller(Px, Py, Pz, gamma))
    {
        // updatePoint();
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        desired_position[i] = angleToPosition(i, desired_angle[i]);
        // SetPosition(id[i], desired_position[i]);
    }
    syncWrite(4);
}

void WidowX::syncWrite(uint8_t numServos)
{
    int temp;
    int length = 4 + (numServos * 3); // 3 = id + pos(2byte)
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_GOAL_POSITION_L);
    ax12write(2);
    for (int i = 0; i < numServos; i++)
    {
        temp = desired_position[i];
        checksum += (temp & 0xff) + (temp >> 8) + id[i];
        ax12write(id[i]);
        ax12write(temp & 0xff);
        ax12write(temp >> 8);
    }
    ax12write(0xff - (checksum % 256));
    setRX(0);
}

//Inverse Kinematics

/**
 * Obtains the IK when Q4 is constants. Returns 0 if succeeds, returns 1 if fails
*/
uint8_t WidowX::getIK_Q4(float Px, float Py, float Pz)
{
    //Obtain q1
    q1 = atan2(Py, Px);

    //Obtain point as seen from {1}
    const float X = sqrt(pow(Px, 2) + pow(Py, 2));
    const float Z = Pz - L0;

    //Read the angle of the fourth motor (q4) and obtain its sine and cosine
    q4 = current_angle[3]; //getServoAngle(3);
    const float s4 = sin(q4), c4 = cos(q4);

    //Calculate the parameters needed to obtain q3
    a = L3 * ca + L4 * ca * c4 + L4 * sa * s4;
    b = L3 * sa - L4 * ca * s4 + L4 * sa * c4;
    c = (pow(X, 2) + pow(Z, 2) - pow(D, 2) - pow(L3, 2) - pow(L4, 2) - 2 * L3 * L4 * c4) / (2 * D);
    cond = pow(a, 2) + pow(b, 2) - pow(c, 2);
    if (cond < 0)
        return 1; //No solution for the IK

    //Obtain q3
    q3 = 2 * atan2(b - sqrt(cond), a + c);
    q3 = atan2(sin(q3), cos(q3));
    uint8_t tryTwice = 1;

    //Check q3 limits
    if (q3 < q3Lim[0] || q3 > q3Lim[1])
    {
        //Try with the other possible solution for q3
        q3 = 2 * atan2(b + sqrt(cond), a + c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 < q3Lim[0] || q3 > q3Lim[1])
            return 1;
        tryTwice = 0;
    }

    float c3, s3;
    for (;;)
    {
        c3 = cos(q3);
        s3 = sin(q3);
        a = D * ca + L3 * c3 + L4 * c3 * c4 - L4 * s3 * s4;
        b = D * sa + L3 * s3 + L4 * s3 * c4 + L4 * c3 * s4;
        q2 = atan2(a * Z - b * X, a * X + b * Z);

        if (q2 < q2Lim[0] || q2 > q2Lim[1])
        {
            if (tryTwice)
            {
                //Try with the other possible solution for q3
                q3 = 2 * atan2(b + sqrt(cond), a + c);
                q3 = atan2(sin(q3), cos(q3));
                if (q3 < q3Lim[0] || q3 > q3Lim[1])
                    //One value of q3 is possible but yields out of range value for q2
                    //and the other value of q3 exceeds q3's limits. Ends function
                    return 1;

                //Check with the other possible value of q3 if there's a solution for q2
                tryTwice = 0;
                continue;
            }
            else
                //Possible value for q2 doesn't exist with any of q3 possible values
                //Ends function
                return 1;
        }
        //Possible value for q2 exists. Breaks loop
        break;
    }

    //Save articular values into the array that will set the next positions
    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;
    desired_angle[3] = q4;
    desired_angle[4] = current_angle[4]; //getServoAngle(4);
    desired_angle[5] = current_angle[5]; //getServoAngle(5);
    //Returns with success
    return 0;
}

/**
 * Obtains the IK with a desired angle gamma for the gripper. 
 * Returns 0 if succeeds, returns 1 if fails
*/
uint8_t WidowX::getIK_Gamma(float Px, float Py, float Pz, float gamma)
{
    //Calculate sine and cosine of gamma
    const float sg = sin(gamma), cg = cos(gamma);

    //Obtain the desired point as seen from {1}
    const float X = sqrt(pow(Px, 2) + pow(Py, 2)) - L4 * cg;
    const float Z = Pz - L0 + L4 * sg;

    //Obtain q1
    q1 = atan2(Py, Px);

    //calculate condition for q3
    c = (pow(X, 2) + pow(Z, 2) - pow(D, 2) - pow(L3, 2)) / (2 * D * L3);

    if (abs(c) > 1)
        return 1;

    q3 = alpha + acos(c);
    q3 = atan2(sin(q3), cos(q3));
    uint8_t tryTwice = 1;

    //Check q3 limits
    if (q3 < q3Lim[0] || q3 > q3Lim[1])
    {
        //Try with the other possible solution for q3
        q3 = alpha - acos(c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 < q3Lim[0] || q3 > q3Lim[1])
            return 1;
        tryTwice = 0;
    }

    float c3, s3;
    for (;;)
    {
        c3 = cos(q3);
        s3 = sin(q3);
        a = D * ca + L3 * c3;
        b = D * sa + L3 * s3;
        q2 = atan2(a * Z - b * X, a * X + b * Z);

        if (q2 < q2Lim[0] || q2 > q2Lim[1])
        {
            if (tryTwice)
            {
                //Try with the other possible solution for q3
                q3 = alpha - acos(c);
                q3 = atan2(sin(q3), cos(q3));
                if (q3 < q3Lim[0] || q3 > q3Lim[1])
                    //One value of q3 is possible but yields out of range value for q2
                    //and the other value of q3 exceeds q3's limits. Ends function
                    return 1;

                //Check with the other possible value of q3 if there's a solution for q2
                tryTwice = 0;
                continue;
            }
            else
                //Possible value for q2 doesn't exist with any of q3 possible values
                //Ends function
                return 1;
        }

        q4 = -gamma - q2 - q3;

        if (q4 < q4Lim[0] || q4 > q4Lim[1])
        {
            if (tryTwice)
            {
                //Try with the other possible solution for q3
                q3 = alpha - acos(c);
                q3 = atan2(sin(q3), cos(q3));
                if (q3 < q3Lim[0] || q3 > q3Lim[1])
                    //One value of q3 is possible but yields out of range value for q4
                    //and the other value of q3 exceeds q3's limits. Ends function
                    return 1;

                //Check with the other possible value of q3 if there's a solution for q4
                tryTwice = 0;
                continue;
            }
            else
                //Possible value for q4 doesn't exist with any of q3 possible values
                //Ends function
                return 1;
        }
        //Possible value for q2 and q4 exists. Breaks loop
        break;
    }

    //Save articular values into the array that will set the next positions
    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;
    desired_angle[3] = q4;
    desired_angle[4] = current_angle[4]; //getServoAngle(4);
    desired_angle[5] = current_angle[5]; //getServoAngle(5);
    //Returns with success
    return 0;
}

uint8_t WidowX::getIK_Rd(float Px, float Py, float Pz, float Rd[3][3])
{
    const float gamma = atan2(-Rd[2][0], Rd[0][0]);

    //Do getIK_Gamma and check if it succeeds. If not, it returns 1
    if (getIK_Gamma(Px, Py, Pz, gamma))
        return 1;

    //Obtain the matrixes to calculate q5
    float rx5_3_2 = Rd[2][1] * cos(gamma) + Rd[0][1] * sin(gamma);
    //float rx5_1_1 = Rd[1][1]
    q5 = atan2(rx5_3_2, Rd[1][1]);

    //Save q5 into the desired_angle array. the other values
    //Have already been saved by getIKGamma
    desired_angle[4] = q5;

    return 0;
}

uint8_t WidowX::getIK_RdBase(float Px, float Py, float Pz, float RdBase[3][3])
{
    //Obtain the desired rotation as seen from {1} to use it with getIK_Rd
    q1 = atan2(Py, Px);

    float c1 = cos(q1), s1 = sin(q1);

    float r11 = RdBase[0][0] * c1 + RdBase[1][0] * s1;
    float r12 = RdBase[0][1] * c1 + RdBase[1][1] * s1;
    float r22 = RdBase[1][1] * c1 - RdBase[0][1] * s1;
    // float r31 = RdBase[2][0];
    // float r32 = RdBase[2][1];

    // RotZ(q1)^-1 * RdBase
    // Only the Rd values that are needed in the getIK_Rd function are calculated
    float Rd[3][3] = {{r11, r12, 0},
                      {0, r22, 0},
                      {RdBase[2][0], RdBase[2][1], 0}};

    return getIK_Rd(Px, Py, Pz, Rd);
}

/**
 * Obtains the IK with a desired angle gamma for the gripper. 
 * Returns 0 if succeeds, returns 1 if fails
*/
uint8_t WidowX::getIK_Gamma_Controller(float Px, float Py, float Pz, float gamma)
{
    //Calculate sine and cosine of gamma
    const float sg = sin(gamma), cg = cos(gamma);

    //Obtain the desired point as seen from {1}
    const float X = sqrt(pow(Px, 2) + pow(Py, 2)) - L4 * cg;
    const float Z = Pz - L0 + L4 * sg;

    //Obtain q1
    q1 = atan2(Py, Px);

    //calculate condition for q3
    c = (pow(X, 2) + pow(Z, 2) - pow(D, 2) - pow(L3, 2)) / (2 * D * L3);

    if (abs(c) > 1)
        return 1;

    float q3_prev = getServoAngle(2);

    float q3_1 = alpha + acos(c);
    q3_1 = atan2(sin(q3_1), cos(q3_1));
    float q3_2 = alpha - acos(c);
    q3_2 = atan2(sin(q3_2), cos(q3_2));

    uint8_t selection = 0;

    if (abs(q3_1 - q3_prev) > abs(q3_2 - q3_prev))
    {
        q3 = q3_2;
        selection = 0;
    }
    else
    {
        q3 = q3_1;
        selection = 1;
    }

    uint8_t tryTwice = 1;

    //Check q3 limits
    if (q3 < q3Lim[0] || q3 > q3Lim[1])
    {
        //Try with the other possible solution for q3
        q3 = alpha + pow(-1, selection) * acos(c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 < q3Lim[0] || q3 > q3Lim[1])
            return 1;
        tryTwice = 0;
    }

    float c3, s3;
    for (;;)
    {
        c3 = cos(q3);
        s3 = sin(q3);
        a = D * ca + L3 * c3;
        b = D * sa + L3 * s3;
        q2 = atan2(a * Z - b * X, a * X + b * Z);

        if (q2 < q2Lim[0] || q2 > q2Lim[1])
        {
            if (tryTwice)
            {
                //Try with the other possible solution for q3
                q3 = alpha + pow(-1, selection) * acos(c);
                q3 = atan2(sin(q3), cos(q3));
                if (q3 < q3Lim[0] || q3 > q3Lim[1])
                    //One value of q3 is possible but yields out of range value for q2
                    //and the other value of q3 exceeds q3's limits. Ends function
                    return 1;

                //Check with the other possible value of q3 if there's a solution for q2
                tryTwice = 0;
                continue;
            }
            else
                //Possible value for q2 doesn't exist with any of q3 possible values
                //Ends function
                return 1;
        }

        q4 = -gamma - q2 - q3;

        if (q4 < q4Lim[0] || q4 > q4Lim[1])
        {
            if (tryTwice)
            {
                //Try with the other possible solution for q3
                q3 = alpha + pow(-1, selection) * acos(c);
                q3 = atan2(sin(q3), cos(q3));
                if (q3 < q3Lim[0] || q3 > q3Lim[1])
                    //One value of q3 is possible but yields out of range value for q4
                    //and the other value of q3 exceeds q3's limits. Ends function
                    return 1;

                //Check with the other possible value of q3 if there's a solution for q4
                tryTwice = 0;
                continue;
            }
            else
                //Possible value for q4 doesn't exist with any of q3 possible values
                //Ends function
                return 1;
        }
        //Possible value for q2 and q4 exists. Breaks loop
        break;
    }

    //Save articular values into the array that will set the next positions
    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;
    desired_angle[3] = q4;
    //Returns with success
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////
