#include <BasicLinearAlgebra.h>
#include <WidowX.h>
#include <poses.h>

WidowX widow = WidowX();
char option;

void setup()
{
    Serial.begin(115200);
    Serial.println("...Starting Robotic Arm...");
    delay(300);
    widow.init(0); //Check voltage, move to rest and do not relax the servos
    delay(100);
    menu();
}

void loop()
{
    if (Serial.available())
    {
        option = Serial.read();
        //Flush serial input
        if (Serial.available())
          Serial.readString();
        switch (option)
        {
        case '0':
            widow.moveRest();
            break;
        case '1':
            widow.moveHome();
            break;
        case '2':
            move2Angle();
            break;
        case '3':
            move2Pos();
            break;
        case '4':
            moveGrip();
            break;
        case '5':
            tryIK();
            break;
        case '6':
          widow.relaxServos();
          break;
        case '7':
          widow.torqueServos();
          break;
        default:
            widow.moveCenter();
            break;
        }

        delay(10);
        if (Serial.available())
            Serial.readString();
        delay(10);
        menu();
    }
}

void move2Angle()
{
  Serial.println("\n***Move Servo 2 Angle***");
  Serial.print("Idx: ");
  while (!Serial.available());
  //Get Idx
  int idx = Serial.parseInt();

  //Flush serial input
  if (Serial.available())
    Serial.readString();
  
  Serial.println(idx);

  //Get desired angle
  Serial.print("Angle: ");
  while (!Serial.available());
  float angle = Serial.parseFloat();
  Serial.println(angle);
  
  widow.moveServo2Angle(idx, angle);
}

void move2Pos()
{
  Serial.println("\n***Move Servo 2 Position***");
  Serial.print("Idx: ");
  while (!Serial.available());
  //Get Idx
  int idx = Serial.parseInt();

  //Flush serial input
  if (Serial.available())
    Serial.readString();
  
  Serial.println(idx);

  //Get desired position
  Serial.print("Position: ");
  while (!Serial.available());
  
  int pos = Serial.parseInt();
  Serial.read();
  Serial.println(pos);
  widow.moveServo2Position(idx, pos);
}

void moveGrip()
{
  Serial.println("\n***Turn Wrist***");
  Serial.print("Close? 0. false, 1. true --> ");
  while (!Serial.available());
  
  int openGrip = Serial.parseInt();

  //Flush serial input
  if (Serial.available())
    Serial.readString();
  
  Serial.println(openGrip);
  Serial.println("Moving... Send any char to stop");
  while (!Serial.available())
  {
    widow.moveGrip(openGrip);
    delay(30);
  }
  Serial.readString();
}

void tryIK()
{
  Serial.println("\n***Sequence to try inverse kinematics***");

  Serial.println("\nIK With Q4");
  Serial.println("Setting Q4 to -pi/4...");
  widow.moveServo2Angle(3, -M_PI_4);
  delay(3000);
  Serial.println("Moving gripper to (10,0,30)...");
  widow.moveArmQ4(10, 0, 30);
  delay(5000);
  Serial.println("Moving gripper to (15,-15,20) in 4s...");
  widow.moveArmQ4(15, -15, 20, 4000);
  delay(5000);

  Serial.println("\nIK With Gamma");
  Serial.println("For Pick N Drop, Gamma = pi/2");
  Serial.println("Moving gripper to (20,0,10)...");
  widow.moveArmGamma(20, 0, 10, M_PI_2);
  delay(5000);
  Serial.println("Moving gripper to (15,15,40) with gamma = -pi/2 in 3s...");
  widow.moveArmGamma(15, 15, 40, -M_PI_2, 3000);
  delay(5000);

  Serial.println("\nIK with Desired Rotation from {1}");
  Serial.println("With a RotX(pi/2) the Q5 should be pi/2 and the gripper will always have a gamma of 0°");
  Matrix<3, 3> Rd;
  widow.rotx(M_PI_2, Rd);
  Serial.println("Moving gripper to (0,0,28)...");
  widow.moveArmRd(0, 0, 28, Rd);
  delay(5000);
  Serial.println("Moving gripper to (15,-10,35) in 3.5s...");
  widow.moveArmRd(15, -10, 35, Rd, 3500);
  delay(5000);
  
  Serial.println("\nIK with Desired Rotation from base");
  Serial.println("With a RotY(-pi/2) the gripper should be looking up and the q5 will be adjusted to represent solely a rotation in 'y'");
  Matrix<3, 3> RdBase;
  widow.roty(-M_PI_2, RdBase);
  Serial.println("Moving gripper to (13,10,40)...");
  widow.moveArmRdBase(13, 10, 40, RdBase);
  delay(5000);

  widow.moveRest();
}

void menu()
{
    Serial.println("\nOptions: ");
    Serial.println("0. Rest");
    Serial.println("1. Home");
    Serial.println("2. Move Servo 2 Angle");
    Serial.println("3. Move Servo 2 Position");
    Serial.println("4. Move Grip");
    Serial.println("5. Try Inverse Kinematics");
    Serial.println("6. Relax Servos");
    Serial.println("7. Torque Servos");
    Serial.println("other. Move Center");
}
