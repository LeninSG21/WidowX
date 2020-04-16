#include <BasicLinearAlgebra.h>

#include <WidowX.h>
#include <poses.h>

using namespace BLA;

WidowX widow = WidowX();

float px, py, pz;
float point[] = {0,0,0};

int option = 1;

void setup()
{
  Serial.begin(9600);
  Serial.println("...Starting Robotic Arm...");
  delay(300);
  
  
  widow.init();
  delay(1000);
  //Serial.print("Px: ");
  menu();
}

void loop() 
{
  if(Serial.available())
  {
    option = Serial.parseInt();
    Serial.read();
    switch(option)
    {
      case 0:
        widow.MoveRest();
        break;
      case 1:
        widow.MoveHome();
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
        moveWrist();
        break;
      case 6:
        turnWrist();
        break;
      case 7:
        moveGrip();
        break;
      default:
        widow.MoveCenter();
        break;
    }
    printPoint();
    menu();
    delay(10);
  }
}

void moveWrist()
{
  Serial.println("\n***Move Wrist***");
  Serial.print("Direction? 0. negative, 1. positive --> ");
  while(!Serial.available());
  int dir = Serial.parseInt();
  Serial.read();
  Serial.println(dir);

  Serial.println("Moving... Send any char to stop");
  while(!Serial.available())
  {
    widow.MoveWrist(dir);
    delay(30);
  }
  Serial.readString();
    
}

void turnWrist()
{
  Serial.println("\n***Turn Wrist***");
  Serial.print("Direction? 0. negative, 1. positive --> ");
  while(!Serial.available());
  int dir = Serial.parseInt();
  Serial.read();
  Serial.println(dir);

  Serial.println("Moving... Send any char to stop");
  while(!Serial.available())
  {
    widow.TurnWrist(dir);
    delay(30);
  }
  Serial.readString();
}

void moveGrip()
{
  Serial.println("\n***Turn Wrist***");
  Serial.print("Close? 0. false, 1. true --> ");
  while(!Serial.available());
  int openGrip = Serial.parseInt();
  Serial.read();
  Serial.println(openGrip);

  Serial.println("Moving... Send any char to stop");
  while(!Serial.available())
  {
    widow.MoveGrip(openGrip);
    delay(30);
  }
  Serial.readString();
}

void move2Point()
{
  Serial.println("\n***Move Servo 2 Point***");
  //Px
  Serial.print("Px: ");
  while(!Serial.available());
  px = Serial.parseFloat();
  Serial.read();
  Serial.println(px);
  
  //Py
  Serial.print("Py: ");
  while(!Serial.available());
  py = Serial.parseFloat();
  Serial.read();
  Serial.println(py);

  //Pz
  Serial.print("Pz: ");
  while(!Serial.available());
  pz = Serial.parseFloat();
  Serial.read();
  Serial.println(pz);
  
  widow.MoveArm(px,py,pz,2500);
 
  
}

void move2Angle()
{
  Serial.println("\n***Move Servo 2 Angle***");
  Serial.print("Id: ");
  while(!Serial.available());
  int id = Serial.parseInt();
  Serial.read();
  Serial.println(id);
  Serial.print("Angle: ");
  while(!Serial.available());
  float angle = Serial.parseFloat();
  Serial.read();
  Serial.println(angle);
  widow.MoveServo2Angle(id, angle);
}

void move2Pos()
{
  Serial.println("\n***Move Servo 2 Position***");
  Serial.print("Id: ");
  while(!Serial.available());
  int id = Serial.parseInt();
  Serial.read();
  Serial.println(id);
  Serial.print("Position: ");
  while(!Serial.available());
  int pos = Serial.parseFloat();
  Serial.read();
  Serial.println(pos);
  widow.MoveServo2Position(id, pos);
}

void printPoint()
{
  widow.getPoint(point);
  Serial.print("X: "); Serial.print(point[0]);
  Serial.print(", Y: "); Serial.print(point[1]);
  Serial.print(", Z: "); Serial.print(point[2]);
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
  Serial.println("other. Move Center");
}
