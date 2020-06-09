#include <BasicLinearAlgebra.h>

#include <WidowX.h>
#include <poses.h>

using namespace BLA;

WidowX widow = WidowX();

float px, py, pz;
float point[] = {0, 0, 0};

int option = 1;

/*
 *  float w[2][4];
  Matrix<4> p1 = {500, 2200, 0, 0};
  Matrix<4> p2 = {4000, 1520, 0, 0};
  widow.cubeInterpolation(p1, w[0], 2);
  widow.cubeInterpolation(p2, w[1], 2);

  delay(10);
  Serial.print("0: {");Serial.print(w[0][0]);Serial.print(", ");
  Serial.print(w[0][1]);Serial.print(", ");
  Serial.print(w[0][2]);Serial.print(", ");
  Serial.print(w[0][3]);Serial.println("}");
  Serial.print("1: {");Serial.print(w[1][0]);Serial.print(", ");
  Serial.print(w[1][1]);Serial.print(", ");
  Serial.print(w[1][2]);Serial.print(", ");
  Serial.print(w[1][3]);Serial.println("}");
 */

void setup()
{
  Serial.begin(9600);
  Serial.println("...Starting Robotic Arm...");
  delay(300);
  widow.init();
  delay(100);
  menu();
}

void loop()
{
  if (Serial.available())
  {
    option = Serial.parseInt();
    Serial.read();
    switch (option)
    {
    case 0:
      widow.moveRest();
      break;
    case 1:
      widow.moveHome();
      break;
    case 2:
      move2Angle();
      break;
    case 3:
      move2Pos();
      break;
    case 4:
      move2Point();
      break;
    case 5:
      MoveWrist();
      break;
    case 6:
      TurnWrist();
      break;
    case 7:
      MoveGrip();
      break;
    case 8:
      TryIK();
      break;
    default:
      widow.moveCenter();
      break;
    }
    printPoint();
    menu();
    delay(10);
  }
}

void TryIK()
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

void MoveWrist()
{
  Serial.println("\n***Move Wrist***");
  Serial.print("Direction? 0. negative, 1. positive --> ");
  while (!Serial.available())
    ;
  int dir = Serial.parseInt();
  Serial.read();
  Serial.println(dir);

  Serial.println("Moving... Send any char to stop");
  while (!Serial.available())
  {
    widow.moveWrist(dir);
    delay(30);
  }
  Serial.readString();
}

void TurnWrist()
{
  Serial.println("\n***Turn Wrist***");
  Serial.print("Direction? 0. negative, 1. positive --> ");
  while (!Serial.available())
    ;
  int dir = Serial.parseInt();
  Serial.read();
  Serial.println(dir);

  Serial.println("Moving... Send any char to stop");
  while (!Serial.available())
  {
    widow.turnWrist(dir);
    delay(30);
  }
  Serial.readString();
}

void MoveGrip()
{
  Serial.println("\n***Turn Wrist***");
  Serial.print("Close? 0. false, 1. true --> ");
  while (!Serial.available())
    ;
  int openGrip = Serial.parseInt();
  Serial.read();
  Serial.println(openGrip);

  Serial.println("Moving... Send any char to stop");
  while (!Serial.available())
  {
    widow.moveGrip(openGrip);
    delay(30);
  }
  Serial.readString();
}

void move2Point()
{
  Serial.println("\n***Move Servo 2 Point***");
  //Px
  Serial.print("Px: ");
  while (!Serial.available())
    ;
  px = Serial.parseFloat();
  Serial.read();
  Serial.println(px);

  //Py
  Serial.print("Py: ");
  while (!Serial.available())
    ;
  py = Serial.parseFloat();
  Serial.read();
  Serial.println(py);

  //Pz
  Serial.print("Pz: ");
  while (!Serial.available())
    ;
  pz = Serial.parseFloat();
  Serial.read();
  Serial.println(pz);

  widow.moveArmQ4(px, py, pz,3000);
}

void move2Angle()
{
  Serial.println("\n***Move Servo 2 Angle***");
  Serial.print("Id: ");
  while (!Serial.available())
    ;
  int id = Serial.parseInt();
  Serial.read();
  Serial.println(id);
  Serial.print("Angle: ");
  while (!Serial.available())
    ;
  float angle = Serial.parseFloat();
  Serial.read();
  Serial.println(angle);
  widow.moveServo2Angle(id, angle);
}

void move2Pos()
{
  Serial.println("\n***Move Servo 2 Position***");
  Serial.print("Id: ");
  while (!Serial.available())
    ;
  int id = Serial.parseInt();
  Serial.read();
  Serial.println(id);
  Serial.print("Position: ");
  while (!Serial.available())
    ;
  int pos = Serial.parseFloat();
  Serial.read();
  Serial.println(pos);
  widow.moveServo2Position(id, pos);
}

void printPoint()
{
  widow.getPoint(point);
  Serial.print("X: ");
  Serial.print(point[0]);
  Serial.print(", Y: ");
  Serial.print(point[1]);
  Serial.print(", Z: ");
  Serial.print(point[2]);
  Serial.println("\n\n");
}

void menu()
{
  Serial.println("Options: ");
  Serial.println("0. Rest");
  Serial.println("1. Home");
  Serial.println("2. Move Servo 2 Angle");
  Serial.println("3. Move Servo 2 Position");
  Serial.println("4. Move Servo 2 Point");
  Serial.println("5. Move Wrist");
  Serial.println("6. Turn Wrist");
  Serial.println("7. Move Grip");
  Serial.println("8. Try Inverse Kinematics");
  Serial.println("other. Move Center");
}
