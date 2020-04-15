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
    //Initializers
    WidowX();
    void init();

    //id Handlers
    void setId(int idx, int newID);
    int getId(int idx);

    //Preloaded Positions
    void moveCenter();
    void moveHome();
    void moveRest();

    //Get Information
    void checkVoltage();
    void getCurrentPosition();
    int getServoPosition(int idx);
    void getPoint(float *p);

    //Torque
    void relaxServos();
    void torqueServos();

    //Move Servo
    void moveServo2Angle(int idx, float angle);
    void moveServo2Position(int idx, int pos);
    void moveWrist(int direction);
    void turnWrist(int direction);
    void moveGrip(int close);

    //Move Arm
    void moveArmQ4(float Px, float Py, float Pz);
    void moveArmQ4(float Px, float Py, float Pz, int time);
    void moveArmGamma(float Px, float Py, float Pz, float gamma);
    void moveArmGamma(float Px, float Py, float Pz, float gamma, int time);
    void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd);
    void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd, int time);
    void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase);
    void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase, int time);

private:
    //Constants
    BioloidController bioloid;
    const uint8_t SERVOCOUNT;
    const float L0, L1, L2, L3, L4, D, alpha;
    const int DEFAULT_TIME;

    //Variables
    uint8_t id[6];
    uint8_t isRelaxed;
    int current_position[6];
    float current_angle[6];
    float desired_angle[6];
    int desired_position[6];
    float point[3];

    //Conversions
    float positionToAngle(int idx, int position);
    int angleToPosition(int idx, float angle);

    //Poses and interpolation
    void interpolate(int time);
    void setBioloidPose();
    void getPoint();

    //Inverse Kinematics
    uint8_t getIK_Q4(float Px, float Py, float Pz);
    uint8_t getIK_Gamma(float Px, float Py, float Pz, float gamma);
    uint8_t getIK_Rd(float Px, float Py, float Pz, Matrix<3, 3> &Rd);
    uint8_t getIK_RdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase);

    //Rotations
    void rotz(float angle, Matrix<3, 3> &Rz);
    void roty(float angle, Matrix<3, 3> &Ry);
    void rotx(float angle, Matrix<3, 3> &Rx);
};

#endif