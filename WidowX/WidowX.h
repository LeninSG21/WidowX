/*
 * WidowX.h - Library to control WidowX Robotic Arm of Trossen Robotics
 * Created by Lenin Silva, April, 2020
 */

#ifndef WidowX_h
#define WidowX_h

#include "Arduino.h"
#include <ax12.h>
#include <BioloidController.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define MX_28 0
#define MX_64 1
#define AX_12 2

class WidowX
{
public:
    WidowX();
    void init();
    void CheckVoltage();
    void MoveCenter();
    void MoveHome();
    void MoveRest();
    void MoveArm(float Px, float Py, float Pz, BLA::Matrix<3, 3> &Rd);
    void MoveArm(float Px, float Py, float Pz, BLA::Matrix<3, 3> &Rd, int time);
    void MoveArm(float Px, float Py, float Pz);
    void MoveArm(float Px, float Py, float Pz, int time);
    void RelaxServos();
    void TorqueServos();
    void MoveServo2Angle(int id, float angle);
    void MoveServo2Position(int id, int pos);
    void MoveWrist(int direction);
    void TurnWrist(int direction);
    void MoveGrip(int close);
    void getCurrentPosition();
    void getPoint(float *p);
    void rotz(float angle, Matrix<3, 3> &Rz);
    void roty(float angle, Matrix<3, 3> &Ry);
    void rotx(float angle, Matrix<3, 3> &Rx);

private:
    //Variables
    BioloidController bioloid;
    const uint8_t SERVOCOUNT;
    uint8_t id;
    uint8_t isRelaxed;
    const float L0, L1, L2, L3, L4, D, alpha;
    const int DEFAULT_TIME;

    int current_position[6];
    float current_angle[6];
    float desired_angle[6];
    int desired_position[6];

    float point[3];

    //Functions
    void getDesiredPosition(int success);
    float positionToAngle(int id, int position);
    int angleToPosition(int id, float angle);
    uint8_t getIK(const float Px, const float Py, const float Pz, Matrix<3, 3> &Rd);
    uint8_t getIK(const float Px, const float Py, const float Pz);
    Matrix<3, 3> getRotZ_Q5(float gamma, float q1, const Matrix<3, 3> &Rd);
    void setBioloidPose();
    void getPoint();

    //Serial functions
    void printArr(float arr[]);
    void printArr(int arr[]);
};

#endif