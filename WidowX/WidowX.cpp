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

const Matrix<3, 3> invRotXPi = {1, 0, 0,
                                0, -1, 0,
                                0, 0, -1};

const Matrix<3, 3> invRotYPi_2 = {0, 0, -1,
                                  0, 1, 0,
                                  1, 0, 0};

Matrix<3, 3> RotYGamma;
Matrix<3, 3> RotZ_Q1;
float a, b, c, cond;
float q1, q2, q3, q4, q5, q6;
float phi2;
int posQ4, posQ5, posQ6;

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

uint8_t WidowX::getIK(const float Px, const float Py, const float Pz)
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
    if (q3 > 8 * M_PI / 9 || q3 < -23 * M_PI / 45)
    {
        q3 = 2 * atan2(b + sqrt(cond), a + c);
        q3 = atan2(sin(q3), cos(q3));
        if (q3 > 8 * M_PI / 9 || q3 < -23 * M_PI / 45)
            return 0;
    }

    //Second aticular value
    const float c3 = cos(q3), s3 = sin(q3);

    a = D * ca + L3 * c3 + L4 * c3 * c4 - L4 * s3 * s4;
    b = D * sa + L3 * s3 + L4 * s3 * c4 + L4 * c3 * s4;
    q2 = atan2(a * Z - b * X, a * X + b * Z);
    if (abs(q2) > 17 * M_PI / 32)
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

uint8_t WidowX::getIK(const float Px, const float Py, const float Pz, Matrix<3, 3> &Rd)
{
    //Transformación de punto deseado de 0 a 5
    float x = Px - L4 * Rd(0, 2), y = Py - L4 * Rd(1, 2), z = Pz - L0 - L4 * Rd(2, 2);

    //Condición para el tercer valor articular
    float b = (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(D, 2) - pow(L3, 2)) / (2 * D * L3);
    if (1 - pow(b, 2) < 0)
        return false;
    //Se obtiene el tercer valor articular
    float q3 = alpha - acos(b);
    //Se revisa la geometría del brazo
    if (q3 > 8 * M_PI / 9 || q3 < -23 * M_PI / 45)
    {
        q3 = alpha - acos(b);
        if (q3 > 8 * M_PI / 9 || q3 < -23 * M_PI / 45)
        {
            return 0;
        }
    }

    //Se obtiene el primer valor articular
    float q1_1 = atan2(y, x), q1_2 = atan2(-y, -x), q1;
    if (abs(q1_1) < abs(q1_2))
        q1 = q1_1;
    else
        q1 = q1_2;

    //Se obtiene el segundo valor articular
    float q2 = atan2(z, x * cos(q1) + y * sin(q1)) - atan2(D * sin(alpha) + L3 * sin(q3), D * cos(alpha) + L3 * cos(q3));

    //Se cargan los tres valores articulares a la matriz
    desired_angle[0] = q1;
    desired_angle[1] = q2;
    desired_angle[2] = q3;

    //Se obtiene gamma de la rotación deseada
    float temp = sqrt(1 - pow(Rd(2, 2), 2)); //variable temporal para no repetir cálculo
    //opciones de valor gamma. Se deben cumplir ambas
    float gS[2] = {atan2(Rd(2, 2), temp), atan2(Rd(2, 2), -temp)}, gC[2];

    float e = 0.01;                        //error permitido
    if (abs(q1) < e || M_PI - abs(q1) < e) // sin(q1) = 0
    {
        gC[0] = acos(Rd(0, 2) / cos(q1));
        gC[1] = -acos(Rd(0, 2) / cos(q1));
    }
    else
    {
        gC[0] = acos(Rd(1, 2) / sin(q1));
        gC[1] = -acos(Rd(1, 2) / sin(q1));
    }

    float gamma;
    //Condición de que ambos resultados sean aproximadamente iguales
    if (abs(gS[0] - gC[0]) <= e || abs(gS[0] - gC[1]) <= e)
        gamma = gS[0];
    else if (abs(gS[1] - gC[0]) <= e || abs(gS[1] - gC[1]) <= e)
        gamma = gS[1];
    else
        return 1;

    //Se obtiene el cuarto valor articular
    float q4 = gamma - q2 - q3;
    if (q4 > 23 * M_PI / 45)
    {
        q4 = M_PI_2;
        gamma = q4 + q2 + q3;
    }
    else if (q4 < -23 * M_PI / 45)
    {
        q4 = -M_PI_2;
        gamma = q4 + q2 + q3;
    }

    //Matriz de rotación de q5 en Z
    Matrix<3, 3> RzQ5 = getRotZ_Q5(gamma, q1, Rd);
    float q5 = atan2(RzQ5(1, 0), RzQ5(0, 0));
    q5 = atan2(sin(q5), cos(q5));

    desired_angle[3] = q4;
    desired_angle[4] = q5;
    desired_angle[5] = 0.0;
    return 2;
}

Matrix<3, 3> WidowX::getRotZ_Q5(float gamma, float q1, const Matrix<3, 3> &Rd)
{
    roty(gamma, RotYGamma); //Calcular RotY(gamma)
    rotz(-q1, RotZ_Q1);     //Calcular RotY(-q1)
    return RotYGamma.Inverse() * invRotYPi_2 * RotZ_Q1.Inverse() * invRotXPi * Rd;
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

void WidowX::printArr(float arr[])
{
    Serial.print("{ ");
    Serial.print(arr[0]);
    for (int i = 1; i < SERVOCOUNT; i++)
    {
        Serial.print(", ");
        Serial.print(arr[i]);
    }
    Serial.println(" }");
}

void WidowX::printArr(int arr[])
{
    Serial.print("{ ");
    Serial.print(arr[0]);
    for (int i = 1; i < SERVOCOUNT; i++)
    {
        Serial.print(", ");
        Serial.print(arr[i]);
    }
    Serial.println(" }");
}

void WidowX::MoveArm(float Px, float Py, float Pz, BLA::Matrix<3, 3> &Rd)
{
    if (isRelaxed)
        TorqueServos();

    bioloid.readPose();
    switch (getIK(Px, Py, Pz, Rd))
    {
    case 0:
        Serial.println("IK cannot be solved!");
        return;
    case 1:
        Serial.println("Couldn't obtain values for wrist (q4 and q5)!");
        getDesiredPosition(0);
        break;
    case 2:
        Serial.println("Moving arm!");
        getDesiredPosition(1);
        break;
    default:
        return;
    }

    bioloid.interpolateSetup(DEFAULT_TIME); // setup for interpolation from current->next over 1/2 a second
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.
        delay(3);
    }
}

//Set a time for interpoalation in miliseconds
void WidowX::MoveArm(float Px, float Py, float Pz, BLA::Matrix<3, 3> &Rd, int time)
{
    if (isRelaxed)
        TorqueServos();

    bioloid.readPose();
    switch (getIK(Px, Py, Pz, Rd))
    {
    case 0:
        Serial.println("IK cannot be solved!");
        return;
    case 1:
        Serial.println("Couldn't obtain values for wrist (q4 and q5)!");
        getDesiredPosition(0);
        break;
    case 2:
        Serial.println("Moving arm!");
        getDesiredPosition(1);
        break;
    default:
        return;
    }
    bioloid.interpolateSetup(time); // setup for interpolation from current->next over 1/2 a second
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.
        delay(3);
    }
}

void WidowX::getDesiredPosition(int success)
{
    if (success)
    {
        for (id = 0; id < SERVOCOUNT; id++)
        {
            desired_position[id] = angleToPosition(id, desired_angle[id]);
            bioloid.setNextPose(id + 1, desired_position[id]);
            // current_position[id] = GetPosition(id + 1);
        }
    }
    else
    {
        for (id = 0; id < 3; id++)
        {
            desired_position[id] = angleToPosition(id, desired_angle[id]);
            bioloid.setNextPose(id + 1, desired_position[id]);
        }
        bioloid.setNextPose(4, bioloid.getCurPose(4));
        bioloid.setNextPose(5, bioloid.getCurPose(5));
    }
}
// void WidowX::getSpeed()
// {

//     int s;
//     uint8_t direction;
//     for (id = 0; id < SERVOCOUNT; id++)
//     {
//         s = ax12GetRegister(id + 1, AX_PRESENT_SPEED_L, 2);
//         direction = s > 1023;
//         s = s % 1024;
//         if (direction)
//             speed[id] = -s * 0.01152; //rad/s = 0.11rpm * (2*pi rad) / 60s
//         else
//             speed[id] = s * 0.01152; //rad/s = 0.11rpm * (2*pi rad) / 60s
//     }
// }
