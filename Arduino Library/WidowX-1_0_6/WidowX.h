/*
WidowX.h - Library to control WidowX Robotic Arm of Trossen Robotics
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

#ifndef WidowX_h
#define WidowX_h

#include "Arduino.h"

#define MX_28 0
#define MX_64 1
#define AX_12 2

#define M_PI_2 1.57079632679489661923132169163975144
#define M_PI_4 0.785398163397448

class WidowX
{
public:
    //INITIALIAZERS
    WidowX();
    void init(uint8_t relax);

    //ID HANDLERS
    void setId(int idx, int newID);
    int getId(int idx);

    //Preloaded Poses
    void moveCenter();
    void moveHome();
    void moveRest();
    void moveToPose(const unsigned int *pose);

    //Get Information
    void checkVoltage();
    void getCurrentPosition();
    void getCurrentPosition(uint8_t until_idx);
    int getServoPosition(int idx);
    float getServoAngle(int idx);
    // void getPoint(float *p);

    //Torque
    void relaxServos();
    void torqueServos();

    //Move Servo
    void moveServo2Angle(int idx, float angle);
    void moveServo2Position(int idx, int pos);
    void moveGrip(int close);
    void openCloseGrip(int close);
    void setServo2Position(int idx, int position);
    void moveServoWithSpeed(int idx, int speed, long initial_time);

    //Move Arm
    void movePointWithSpeed(int vx, int vy, int vz, int vg, long initial_time);
    void moveArmWithSpeed(int vx, int vy, int vz, int vg, long initial_time);
    void moveArmQ4(float Px, float Py, float Pz);
    void moveArmQ4(float Px, float Py, float Pz, int time);
    void moveArmGamma(float Px, float Py, float Pz, float gamma);
    void moveArmGamma(float Px, float Py, float Pz, float gamma, int time);
    void moveArmRd(float Px, float Py, float Pz, float Rd[3][3]);
    void moveArmRd(float Px, float Py, float Pz, float Rd[3][3], int time);
    void moveArmRdBase(float Px, float Py, float Pz, float RdBase[3][3]);
    void moveArmRdBase(float Px, float Py, float Pz, float RdBase[3][3], int time);

    //Sequence
    void performSequenceGamma(float **seq, int num_poses);

    //Rotations
    void rotz(float angle, float Rz[3][3]);
    void roty(float angle, float Ry[3][3]);
    void rotx(float angle, float Rx[3][3]);

private:
    //Constants
    const uint8_t SERVOCOUNT;
    const float L0, L1, L2, L3, L4, D, alpha;
    const float sa, ca; //sin(alpha), cos(alpha);
    const int DEFAULT_TIME;
    //Limits
    const float xy_lim;     // = 43.0;
    const float z_lim_up;   // = 52.0;
    const float z_lim_down; // = -26.0;
    const float gamma_lim;  // = M_PI_2;

    //Multiplying factors
    const float Kp; //[cm/(ms*bit)]
    const float Kg; //[rad/(ms*bit)]
    const float Ks; //[pos/(ms*bit)]

    //Variables
    uint8_t id[6];
    uint8_t isRelaxed;
    uint16_t current_position[6];
    float float_position[6];
    float current_angle[6];
    float desired_angle[6];
    uint16_t desired_position[6];
    uint16_t next_position[6];
    float point[3];
    float speed_points[3];
    float global_gamma;
    float W[6][4];

    //Conversions
    float positionToAngle(int idx, int position);
    int angleToPosition(int idx, float angle);

    //Poses and interpolation
    // void updatePoint();
    void cubeInterpolation(float q0, float qf, float *w, int time);
    void interpolate(int remTime);
    void interpolateFromPose(const unsigned int *pose, int remTime);
    void setArmGamma(float Px, float Py, float Pz, float gamma);
    void syncWrite(uint8_t numServos);

    //Inverse Kinematics
    uint8_t getIK_Q4(float Px, float Py, float Pz);
    uint8_t getIK_Gamma(float Px, float Py, float Pz, float gamma);
    uint8_t getIK_Rd(float Px, float Py, float Pz, float Rd[3][3]);
    uint8_t getIK_RdBase(float Px, float Py, float Pz, float RdBase[3][3]);
    uint8_t getIK_Gamma_Controller(float Px, float Py, float Pz, float gamma);
};

#endif
