/*
 * WidowX.h - Library to control WidowX Robotic Arm of Trossen Robotics
 * Created by Lenin Silva, April, 2020
 * This library uses the index (idx) of each servo instead of its id. This is to simplify
 * The logic. However, since both BioloidController and ax12 need the id of the 
 * motor, an array of ids is also created with the default values from 1 to 6.
 *  
 * When working with idx, the first motor at the base is considered idx = 0; the second
 * motor is idx = 1 and so on until the sixth motor (the gripper) with idx = 5. It is
 * common to set these motors to consecutive ids from 1 to 6. Hence, the id array would 
 * look like this: id = {1,2,3,4,5,6}. However, this would cause problems if the motors
 * were given different arrays, which is why the idx method is preferred. In order to 
 * work with another id, the following must be done:
 * 
 * Let's say that we want the third motor to have an id of 16. The third motor has
 * and idx = 2, so id[2] should be 16. To do that, we use the function setId() and
 * we give it the idx of the motor to change and the new id, like this: setId(2,16).
 * This will also change the id inside the bioloid controller, since this one also
 * needs the id array to be handled properly. To check that the id was set correctly,
 * the getId function can be used: getId(2) --> 16. If the function returns the expected
 * if, you're good to go.
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
    void setServo2Position(int idx, int position);
    void moveServoWithSpeed(int idx, int speed, long initial_time);

    //Move Arm

    void movePointWithSpeed(int vx, int vy, int vz, int vg, long initial_time);
    void moveArmWithSpeed(int vx, int vy, int vz, int vg, long initial_time);
    void moveArmQ4(float Px, float Py, float Pz);
    void moveArmQ4(float Px, float Py, float Pz, int time);
    void moveArmGamma(float Px, float Py, float Pz, float gamma);
    void moveArmGamma(float Px, float Py, float Pz, float gamma, int time);
    void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd);
    void moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd, int time);
    void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase);
    void moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase, int time);

    //Rotations
    void rotz(float angle, Matrix<3, 3> &Rz);
    void roty(float angle, Matrix<3, 3> &Ry);
    void rotx(float angle, Matrix<3, 3> &Rx);

private:
    //Constants
    const uint8_t SERVOCOUNT;
    const float L0, L1, L2, L3, L4, D, alpha;
    const float sa, ca; //sin(alpha), cos(alpha);
    const int DEFAULT_TIME;
    //Limits
    const float xy_lim = 43.0;
    const float z_lim_up = 52.0;
    const float z_lim_down = -26.0;
    const float gamma_lim = M_PI_2;

    //Multiplying factors
    const float Kp = 60.0 / 127000;   //[cm/(ms*bit)]
    const float Kg = M_PI_2 / 255000; //[rad/(ms*bit)]
    const float Ks = 1024.0 / 255000; //[pos/(ms*bit)]

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
    void updatePoint();
    void cubeInterpolation(Matrix<4> &params, float *w, int time);
    void interpolate(int remainingTime);
    void interpolateFromPose(const unsigned int *pose, int remainingTime);
    void setArmGamma(float Px, float Py, float Pz, float gamma);
    void syncWrite(uint8_t numServos);

    //Inverse Kinematics
    uint8_t getIK_Q4(float Px, float Py, float Pz);
    uint8_t getIK_Gamma(float Px, float Py, float Pz, float gamma);
    uint8_t getIK_Rd(float Px, float Py, float Pz, Matrix<3, 3> &Rd);
    uint8_t getIK_RdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase);
    uint8_t getIK_Gamma_Controller(float Px, float Py, float Pz, float gamma);
};

#endif