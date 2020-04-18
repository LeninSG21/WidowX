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
 * if, you're good to go. Howevere, if the return value is -1, it means that there was a 
 * problem when setting the id of the bioloid controller, so the setId operation should 
 * be done once again.
 */
#include "Arduino.h"
#include "WidowX.h"
#include <ax12.h>
#include <BioloidController.h>
#include "math.h"
#include "poses.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

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
int curr_2, curr_3;

//////////////////////////////////////////////////////////////////////////////////////
/*
    *** PUBLIC FUNCTIONS ***
*/

//INITIALIAZERS
/*
 * Constructor: intializes the bioloid controller to a baud of 1000000, sets the
 * servo count to 6, and fills the id array from 1 to 6.
*/
WidowX::WidowX()
    : bioloid(BioloidController(1000000)), SERVOCOUNT(6), L0(9), L1(14), L2(5), L3(14),
      L4(14), D(sqrt(pow(L1, 2) + pow(L2, 2))), alpha(atan2(L1, L2)), sa(sin(alpha)),
      ca(cos(alpha)), isRelaxed(0), DEFAULT_TIME(2000)
{
    bioloid.setup(SERVOCOUNT);
    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        id[i] = i + 1;
    }
}

/*
 * This is NOT a required function in order to work properly. It sends the arm to 
 * rest position and it disables the torque to save energy. It also checks the 
 * voltage with the function checkVoltage(). 
*/
void WidowX::init()
{
    delay(10);
    checkVoltage();
    moveRest();
    delay(100);
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
    bioloid.setId(idx, newID);
}

/*
 * Gets the id of the specified motor by idx. It checks that both the bioloid
 * controller and the id array at the given index have the same value, to assure
 * that it works properly. If both value are the same, it returns the id; 
 * however, if they are different it returns -1 and the setId operation should be 
 * tried once again
*/
int WidowX::getId(int idx)
{
    if (bioloid.getId(idx) == id[idx])
        return id[idx];
    else
        return -1;
}

//Preloaded Positions
/*
 * Moves to the arm to the center of all motors, forming an upside L. As seen from 
 * the kinematic analysis done for this library, it would be equal to setting al
 * articular values to 0°. Gets the current position once it's done.
*/
void WidowX::moveCenter()
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
/*
 * Moves to the arm to the home position as defined by the bioloid controller.
 * Gets the current position once it's done.
*/
void WidowX::moveHome()
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

/*
 * Moves to the arm to the rest position, which is when the arm is resting over itself.
 * Gets the current position once it's done.
 * 
*/
void WidowX::moveRest()
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

//Get Information
/* 
 * Checks that the voltage values are adequate for the robotic arm. If it is below
 * 10V, it remains in a loop until the voltage increases. This is to prevent
 * damage to the arm. Also, it prints to the serial monitor some information about the
 * voltage.
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

/*
 * This function calls the GetPosition function from the ax12.h library. However, it was seen that
 * in some cases the value returned was -1. Hence, it made the arm to move drastically, which 
 * represents a hazar to those around and the arm itself. That is why this function checks if the
 * returned value is -1. If it is, it tries to read the current positon of the servo up to five
 * mover times. If in those tries the value is still -1, then the postion is assumed to be 0.
 * That way, the movement will be better than with the position at -1. Due to this check conditions,+
 * this function is preferred over the readPose() function from the BioloidController.h library.
 * NOTE: Since this function is not used in the predefined movement positions (moveHome, moveCenter and moveRest),
 * there is a chance that the arm will move drastically since the bioloid function does not check 
 * said condition.
*/
int WidowX::getServoPosition(int idx)
{
    uint8_t i = 1;
    current_position[idx] = GetPosition(id[idx]);
    if (current_position[idx] == -1)
    {
        for (i = 1; i < 6; i++)
        {
            current_position[idx] = GetPosition(id[idx]);
            if (current_position[idx] != -1)
                break;
            delay(10 * i);
        }
        if (i == 6)
            current_position[idx] = angleToPosition(idx, 0);
    }
    current_angle[idx] = positionToAngle(idx, current_position[idx]);
    return current_position[idx];
}

/**
 * This function returns the current angle of the specified motor.
 * It uses getServoPosition() to prevent failure from reading -1 in the current
 * position
*/
float WidowX::getServoAngle(int idx)
{
    current_angle[idx] = positionToAngle(idx, getServoPosition(idx));
    return current_angle[idx];
}

/*
 * Calls the private function get point to load the current point and copies
 * the values into the provided pointer
 */
void WidowX::getPoint(float *p)
{
    getPoint();
    p[0] = point[0];
    p[1] = point[1];
    p[2] = point[2];
}

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
            delay(3);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id[idx], --curr);
            delay(3);
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
            delay(3);
        }
    }
    else
    {
        while (curr > pos)
        {
            SetPosition(id[idx], --curr);
            delay(3);
        }
    }
}

/**
 * Moves the wrist (Q4) in the direction specified: 0 --> CW, 1 --> CCW in steps of 50
 * Use it inside a loop with a delay to control the smoothnes of the turn. Ideal for
 * movement with control or key that is being sent as long as it is pressed
*/
void WidowX::moveWrist(int direction)
{
    posQ4 = getServoPosition(3);
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
    SetPosition(id[3], posQ4);
}

/**
 * Turns the wrist (Q5) in the direction specified: 0 --> CW, 1 --> CCW in steps of 10
 * Use it inside a loop with a delay to control the smoothnes of the turn. Ideal for
 * movement with control or key that is being sent as long as it is pressed
*/
void WidowX::turnWrist(int direction)
{
    posQ5 = getServoPosition(4);
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
    SetPosition(id[4], posQ5);
}

/**
 * Closes or opens the gripper (Q6): close = 0 --> open, close = 1 --> close in steps of 10
 * Use it inside a loop with a delay to control the smoothnes of the turn. Ideal for
 * movement with control or key that is being sent as long as it is pressed
*/
void WidowX::moveGrip(int close)
{
    posQ6 = getServoPosition(5);
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
    SetPosition(id[5], posQ6);
}

//Move Arm
/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot.
 * It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function
 * only affects Q1, Q2 and Q3. It interpolates the step using the bioloid interpolation with the default time of 2000 ms.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmQ4(float Px, float Py, float Pz)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Q4(Px, Py, Pz))
    {
        Serial.println("No solution for IK!");
        return;
    }
    // interpolate(DEFAULT_TIME);
    bioloidInterpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot.
 * It uses getIK_Q4, so it moves the arm while maintaining the position of the fourth motor (wrist). This function
 * only affects Q1, Q2 and Q3. It interpolates the step using the bioloid interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmQ4(float Px, float Py, float Pz, int time)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Q4(Px, Py, Pz))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(time);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
 * desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
 * to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
 * It interpolates the step using the bioloid interpolation with the default time of 2000 ms.
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

    bioloidInterpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
 * desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
 * to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
 * It interpolates the step using the bioloid interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmGamma(float Px, float Py, float Pz, float gamma, int time)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Gamma(Px, Py, Pz, gamma))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(time);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matriz of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using the bioloid interpolation with the default time of 2000 ms.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Rd(Px, Py, Pz, Rd))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the coordinate system {1} of the robot. For example, when Rd is an identity matriz of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using the bioloid interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmRd(float Px, float Py, float Pz, Matrix<3, 3> &Rd, int time)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_Rd(Px, Py, Pz, Rd))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(time);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the base of the robot. For example, when Rd is an identity matriz of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using the bioloid interpolation with the default time of 2000 ms.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_RdBase(Px, Py, Pz, RdBase))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(DEFAULT_TIME);
}

/**
 * Moves the center of the gripper to the specified coordinates Px, Py and Pz, and with the desired rotation of the coordinate system
 * of the gripper, as seen from the base of the robot. For example, when Rd is an identity matriz of 3x3, the gripper's 
 * rotation will be such that the orientation of the system {1} and the orientation of the gripper's system will be exactly the same. Thus,
 * it is harder to obtain solutions for the IK. It uses getIK_Rd. This function affects Q1, Q2, Q3, Q4, and Q5. 
 * It interpolates the step using the bioloid interpolation with the given time in milliseconds.
 * If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
*/
void WidowX::moveArmRdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase, int time)
{
    if (isRelaxed)
        torqueServos();

    getCurrentPosition();
    if (getIK_RdBase(Px, Py, Pz, RdBase))
    {
        Serial.println("No solution for IK!");
        return;
    }

    bioloidInterpolate(time);
}

//Rotations
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
void WidowX::bioloidInterpolate(int time)
{
    setBioloidPose();
    delay(10);
    bioloid.readPose();
    delay(50);
    bioloid.interpolateSetup(time);
    while (bioloid.interpolating > 0)
    {                              // do this while we have not reached our new pose
        bioloid.interpolateStep(); // move servos, if necessary.
        delay(3);
    }
}

void WidowX::setBioloidPose()
{
    for (uint8_t i = 0; i < SERVOCOUNT; i++)
    {
        desired_position[i] = angleToPosition(i, desired_angle[i]);
        bioloid.setNextPose(id[i], desired_position[i]);
    }
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

Matrix<4> WidowX::cubeInterpolation(Matrix<4> &params, int time)
{
    const float tf_1_2 = 1 / pow(time, 2);
    const float tf_2_3 = 2 / pow(time, 3);
    const float tf_3_2 = 3 * tf_1_2;

    Matrix<4, 4> M_inv = {1, 0, 0, 0,
                          0, 0, 1, 0,
                          -tf_3_2, tf_3_2, -2 / time, -1 / time,
                          tf_2_3, -tf_2_3, tf_1_2, tf_1_2};
    return M_inv * params;
}

void WidowX::interpolate(int remTime)
{
    uint8_t i;
    for (i = 0; i < SERVOCOUNT - 1; i++)
    {
        desired_position[i] = angleToPosition(i, desired_angle[i]);
        a[i] = cubeInterpolation({current_position[i], desired_position[i], 0, 0}, remainingTime);
    }

    t0 = millis();
    currentTime = millis() - t0;
    while (currentTime < remTime)
    {
        curr_2 = pow(currentTime, 2);
        curr_3 = pow(currentTime, 3);
        for (i = 0; i < SERVOCOUNT - 1; i++)
        {
            next_position[i] = round(a[i](0) + a[i](1) * currentTime + a[i](2) * curr_2 + a[i](3) * curr_3);
        }
        Serial.print("Next Q1 --> ");
        Serial.println(next_position[0]);
        Serial.print("Next Q2 --> ");
        Serial.println(next_position[1]);
        Serial.print("Next Q3 --> ");
        Serial.println(next_position[2]);
        Serial.print("Next Q4 --> ");
        Serial.println(next_position[3]);
        Serial.print("Next Q5 --> ");
        Serial.println(next_position[4]);
        // SetPosition(id[0], next_position[0]);
        // SetPosition(id[1], next_position[1]);
        // SetPosition(id[2], next_position[2]);
        // SetPosition(id[3], next_position[3]);
        // SetPosition(id[4], next_position[4]);
        delay(3);
        currentTime = millis() - t0;
    }

    Serial.println("Finally...");
    Serial.print("Next Q1 --> ");
    Serial.println(next_position[0]);
    Serial.print("Next Q2 --> ");
    Serial.println(next_position[1]);
    Serial.print("Next Q3 --> ");
    Serial.println(next_position[2]);
    Serial.print("Next Q4 --> ");
    Serial.println(next_position[3]);
    Serial.print("Next Q5 --> ");
    Serial.println(next_position[4]);

    // SetPosition(id[0], desired_position[0]);
    // SetPosition(id[1], desired_position[1]);
    // SetPosition(id[2], desired_position[2]);
    // SetPosition(id[3], desired_position[3]);
    // SetPosition(id[4], desired_position[4]);
    delay(3);
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
    desired_angle[4] = current_angle[5]; //getServoAngle(4);
    desired_angle[5] = current_angle[6]; //getServoAngle(5);
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
    desired_angle[4] = current_angle[5]; //getServoAngle(4);
    desired_angle[5] = current_angle[6]; //getServoAngle(5);
    //Returns with success
    return 0;
}

uint8_t WidowX::getIK_Rd(float Px, float Py, float Pz, Matrix<3, 3> &Rd)
{
    const float gamma = atan2(-Rd(2, 0), Rd(0, 0));

    //Do getIK_Gamma and check if it succeeds. If not, it returns 1
    if (getIK_Gamma(Px, Py, Pz, gamma))
        return 1;

    //Obtain the matrixes to calculate q5
    Matrix<3, 3> RyGamma;
    roty(gamma, RyGamma);
    Invert(RyGamma);
    Matrix<3, 3> Rx5 = RyGamma * Rd;
    q5 = atan2(Rx5(2, 1), Rx5(1, 1));

    //Save q5 into the desired_angle array. the other values
    //Have already been saved by getIKGamma
    desired_angle[4] = q5;

    return 0;
}

uint8_t WidowX::getIK_RdBase(float Px, float Py, float Pz, Matrix<3, 3> &RdBase)
{
    //Obtain the desired rotation as seen from {1} to use it with getIK_Rd
    q1 = atan2(Py, Px);
    Matrix<3, 3> RzQ1;
    rotz(q1, RzQ1);
    Invert(RzQ1);
    Matrix<3, 3> Rd = RzQ1 * RdBase;

    return getIK_Rd(Px, Py, Pz, Rd);
}

//////////////////////////////////////////////////////////////////////////////////////