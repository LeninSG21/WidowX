/*
 * WidowX.h - Library to control WidowX Robotic Arm of Trossen Robotics
 * Created by Lenin Silva, April, 2020
 */

#include "Arduino.h"
#include "WidowX.h"
#include <ax12.h>
#include <BioloidController.h>
#include "math.h"
#include "poses.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

Matrix<3, 3> RotYGamma;
Matrix<3, 3> RotZ_Q1;
float a, b, c, cond;
float q1, q2, q3, q4, q5, q6;
float phi2;
int posQ4, posQ5, posQ6;
const limPi_2 = 181 * M_PI / 360;

WidowX::WidowX()
    : bioloid(BioloidController(1000000)), SERVOCOUNT(6), id(1), L0(9), L1(14), L2(5), L3(14),
      L4(14), D(sqrt(pow(L1, 2) + pow(L2, 2))), alpha(atan2(L1, L2)),
      isRelaxed(0), DEFAULT_TIME(1000)
{
    bioloid.setup(SERVOCOUNT);
}

void WidowX::init()
{
    delay(10);

    CheckVoltage();

    MoveRest();
    getCurrentPosition();
    delay(10);
    RelaxServos();
}

void WidowX::CheckVoltage()
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

void WidowX::MoveCenter()
{
    delay(100);               // recommended pause
    bioloid.loadPose(Center); // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();       // read in current servo positions to the curPose buffer
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.

        delay(3);
    }
    getCurrentPosition();
}

void WidowX::MoveHome()
{
    delay(100);             // recommended pause
    bioloid.loadPose(Home); // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();     // read in current servo positions to the curPose buffer
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.

        delay(3);
    }
    getCurrentPosition();
}

void WidowX::MoveRest()
{
    delay(100);             // recommended pause
    bioloid.loadPose(Rest); // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();     // read in current servo positions to the curPose buffer
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.

        delay(3);
    }
    getCurrentPosition();
}

void WidowX::RelaxServos()
{
    for (id = 1; id <= SERVOCOUNT; id++)
    {
        Relax(id);
        delay(10);
    }
    isRelaxed = 1;
}

void WidowX::TorqueServos()
{
    for (id = 1; id <= SERVOCOUNT; id++)
    {
        TorqueOn(id);
        delay(10);
    }
    isRelaxed = 0;
}

void WidowX::MoveServo2Angle(int id, float angle)
{
    if (id < 1 || id > SERVOCOUNT)
        return;
    int pos = angleToPosition(id - 1, angle);
    int curr = GetPosition(id);
    if (curr < pos)
    {
        while (curr < pos)
        {
            SetPosition(id, ++curr);
            delay(3);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id, --curr);
            delay(3);
        }
    }
}

void WidowX::MoveServo2Position(int id, int pos)
{
    if (id < 1 || id > SERVOCOUNT)
        return;
    int curr = GetPosition(id);
    if (curr < pos)
    {
        while (curr < pos)
        {
            SetPosition(id, ++curr);
            delay(3);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id, --curr);
            delay(3);
        }
    }
}

void WidowX::MoveWrist(int direction)
{
    posQ4 = GetPosition(4);
    if (direction)
    {
        if (posQ4 < 3080)
        {
            posQ4 += 50;
        }
    }
    else
    {
        if (posQ4 > 1020)
        {
            posQ4 -= 50;
        }
    }
    SetPosition(4, posQ4);
}

void WidowX::TurnWrist(int direction)
{
    posQ5 = GetPosition(5);
    if (direction)
    {
        if (posQ5 < 1013)
        {
            posQ5 += 10;
        }
        else
        {
            posQ5 = 1023;
        }
    }
    else
    {
        if (posQ5 > 10)
        {
            posQ5 -= 10;
        }
        else
        {
            posQ5 = 0;
        }
    }
    SetPosition(5, posQ5);
}

void WidowX::MoveGrip(int close)
{
    posQ6 = GetPosition(6);
    if (close)
    {
        if (posQ6 < 10)
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
    SetPosition(6, posQ6);
}

void WidowX::MoveArm(float Px, float Py, float Pz)
{
    if (isRelaxed)
        TorqueServos();
    if (getIK(Px, Py, Pz) == 0)
        return;

    setBioloidPose();
    bioloid.interpolateSetup(DEFAULT_TIME);
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.
        delay(3);
    }
}

void WidowX::MoveArm(float Px, float Py, float Pz, int time)
{
    if (isRelaxed)
        TorqueServos();

    if (getIK(Px, Py, Pz) == 0)
    {
        Serial.println("No solution for IK!");
        return;
    }

    setBioloidPose();
    bioloid.interpolateSetup(time);
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.
        delay(3);
    }
}

void WidowX::setBioloidPose()
{
    for (id = 0; id < SERVOCOUNT; id++)
    {
        desired_position[id] = angleToPosition(id, desired_angle[id]);
        bioloid.setNextPose(id + 1, desired_position[id]);
    }
}

void WidowX::getCurrentPosition()
{
    uint8_t i;
    for (id = 0; id < SERVOCOUNT; id++)
    {
        current_position[id] = GetPosition(id + 1);
        if (current_position[id] == -1)
        {
            for (i = 0; i < 5; i++)
            {
                current_position[id] = GetPosition(id + 1);
                if (current_position[id] != -1)
                    break;
                delay(3);
            }
            if (i == 5)
                current_position[id] = angleToPosition(id, 0);
        }
        current_angle[id] = positionToAngle(id, current_position[id]);
    }
}

float WidowX::positionToAngle(int id, int position)
{
    if (id == 0 || id == 3 || id == 2) //MX-28 || MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        // return (float)0.0015 * position - 3.072;
        return 0.00153435538637 * (position - 2047.5);
    else if (id == 1) //MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        // return (float)-0.0015 * position + 3.072;
        return -0.00153435538637 * (position - 2047.5);
    else //AX-12
        //0-1023
        //0° - 300°
        // return (float)0.0051 * position - 2.6112;
        return 0.00511826979472 * (position - 511.5);
}

int WidowX::angleToPosition(int id, float angle)
{
    if (id == 0 || id == 3 || id == 2) //MX-28 || MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        // return (int)651.74 * angle + 2048;
        return round(651.739492 * angle + 2047.5);

    else if (id == 1) //MX-64
        //0 - 4095, 0.088°
        // 0° - 360°
        return round(-651.739492 * angle + 2047.5);
    else //AX-12
        //0-1023
        //0° - 300°
        return round(195.378524405 * angle + 511.5);
}

// int WidowX::getPosQ4()
// {
//     uint8_t i = 0;
//     current_position[3] = GetPosition(4);
//     if (current_position[3] == -1)
//     {
//         for (i = 0; i < 5; i++)
//         {
//             current_position[3] = GetPosition(4);
//             if (current_position[3] != -1)
//                 break;
//             delay(3);
//         }
//         if (i == 5)
//             current_position[3] == = angleToPosition(3, 0);
//     }
//     current_angle[3] = positionToAngle(3, current_position[3]);
// }

uint8_t WidowX::getIK_Q4(const float Px, const float Py, const float Pz)
{

    const float X = sqrt(pow(Px, 2) + pow(Py, 2));
    const float Z = Pz - L0;

    getCurrentPosition();
    q4 = current_angle[3];
    q5 = current_angle[4];
    q6 = current_angle[5];

    const float sa = sin(alpha), ca = cos(alpha);
    const float s4 = sin(q4), c4 = cos(q4);

    //Third articular value
    a = L3 * ca + L4 * ca * c4 + L4 * sa * s4;
    b = L3 * sa - L4 * ca * s4 + L4 * sa * c4;
    c = (pow(X, 2) + pow(Z, 2) - pow(D, 2) - pow(L3, 2) - pow(L4, 2) - 2 * L3 * L4 * c4) / (2 * D);
    cond = pow(a, 2) + pow(b, 2) - pow(c, 2);
    if (cond < 0) //No solution to IK
        return 0;

    q3 = 2 * atan2(b - sqrt(cond), a + c);
    q3 = atan2(sin(q3), cos(q3));
    if (q3 > 3 * M_PI_4 || q3 < -limPi_2)
    {
        q3 = 2 * atan2(b + sqrt(cond), a + c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 > 3 * M_PI_4 || q3 < -limPi_2)
            return 0;
    }

    //Second aticular value
    const float c3 = cos(q3), s3 = sin(q3);

    a = D * ca + L3 * c3 + L4 * c3 * c4 - L4 * s3 * s4;
    b = D * sa + L3 * s3 + L4 * s3 * c4 + L4 * c3 * s4;
    q2 = atan2(a * Z - b * X, a * X + b * Z);
    if (abs(q2) > limPi_2)
        return 0;

    q1 = atan2(Py, Px);

    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;
    desired_angle[3] = q4;
    desired_angle[4] = q5;
    desired_angle[5] = q6;

    return 1;
}

uint8_t WidowX::getIK_Gamma(float Px, float Py, float Pz, float gamma)
{
    float X = sqrt(pow(Px, 2) + pow(Py, 2));
    float Z = Pz - L0;

    const float sa = sin(alpha), ca = cos(alpha);
    const float sg = sin(gamma), cg = cos(gamma);

    X = X - L4 * cg;
    Z = Z + L4 * sg;

    c = (pow(X, 2) + pow(Z, 2) - pow(D, 2) - pow(L3, 2)) / (2 * D * L3);
    if (abs(c) > 1)
        return 0;

    q3 = alpha + acos(c);
    q3 = atan2(sin(q3), cos(q3));

    if (q3 > 3 * M_PI_4 || q3 < -limPi_2)
    {
        q3 = alpha - acos(c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 > 3 * M_PI_4 || q3 < -limPi_2)
            return 0; //No hay solución por la geometría del brazo
    }

    const float c3 = cos(q3), s3 = sin(q3);
    a = D * ca + L3 * c3;
    b = D * sa + L3 * s3;
    q2 = atan2(a * Z - b * X, a * X + b * Z);
    if (abs(q2) > limPi_2)
        return 0;
    q4 = -gamma - q2 - q3;
    if (abs(q4) > limPi_2)
        return 0;

    q1 = atan2(Py, Px);

    getCurrentPosition();
    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;
    desired_angle[3] = q4;
    desired_angle[4] = current_angle[4];
    desired_angle[5] = current_angle[5];

    return 1;
}

void WidowX::getPoint(float *p)
{
    getCurrentPosition();
    q1 = current_angle[0];
    q2 = current_angle[1];
    q3 = current_angle[2];
    q4 = current_angle[3];

    phi2 = D * cos(alpha + q2) + L3 * cos(q2 + q3) + L4 * cos(q2 + q3 + q4);

    point[0] = cos(q1) * phi2;
    point[1] = sin(q1) * phi2;
    point[2] = L0 + D * sin(alpha + q2) + L3 * sin(q2 + q3) + L4 * sin(q2 + q3 + q4);
    p[0] = point[0];
    p[1] = point[1];
    p[2] = point[2];
}

void WidowX::getPoint()
{
    getCurrentPosition();
    q1 = current_angle[0];
    q2 = current_angle[1];
    q3 = current_angle[2];
    q4 = current_angle[3];

    phi2 = D * cos(alpha + q2) + L3 * cos(q2 + q3) + L4 * cos(q2 + q3 + q4);

    point[0] = cos(q1) * phi2;
    point[1] = sin(q1) * phi2;
    point[2] = L0 + D * sin(alpha + q2) + L3 * sin(q2 + q3) + L4 * sin(q2 + q3 + q4);
}

void WidowX::rotz(float angle, Matrix<3, 3> &Rz)
{
    Rz = {cos(angle), -sin(angle), 0,
          sin(angle), cos(angle), 0,
          0, 0, 1};
}

void WidowX::roty(float angle, Matrix<3, 3> &Ry)
{
    Ry = {cos(angle), 0, sin(angle),
          0, 1, 0,
          -sin(angle), 0, cos(angle)};
}

void WidowX::rotx(float angle, Matrix<3, 3> &Rx)
{
    Rx = {1, 0, 0,
          0, cos(angle), -sin(angle),
          0, sin(angle), cos(angle)};
}
