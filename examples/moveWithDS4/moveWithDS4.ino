#include <WidowX.h>

WidowX widow = WidowX();
uint8_t buff[6];
uint8_t open_close, options;
int vx, vy, vz, vg, vq5;
long initial_time;

/*
    BUFFER MESSAGE

    byte index --> data
    0 --> speed in x, where MSb is sign
    1 --> speed in y, where MSb is sign
    2 --> speed in z, where MSb is sign
    3 --> speed for gamma
    4 --> speed for q5
    5 --> Sg<<7 | Sq5 <<6 | open_close[1..0] << 4 | options[3..0]
            Sg --> sign of gamma speed
            Sq5 --> sing of Q5 speed
            open_close: 0b00 || 0b11 --> void
                        0b01 --> open
                        0b10 --> close
            options: 1 --> rest
                    2 --> home
                    3 --> center
                    4 --> relax
                    5 --> torque
                    other --> void
*/

void setup()
{
    Serial.begin(115200);
    Serial.println("...Starting Robotic Arm...");
    delay(300);
    widow.init(0);
    delay(100);
    Serial.println("ok");
    while (!Serial.available());
    while (Serial.readStringUntil('\n') != "ok")
    {
        while (!Serial.available());
    }
    delay(1000);
}

void loop()
{
    if (Serial.available())
    {
        Serial.readBytes(buff,6);
        initial_time = millis();

        options = buff[5] & 0xF;
        if (options == 0 || options > 5)
        {
            vx = buff[0] & 0x7F;
            if (buff[0] >> 7)
                vx = -vx;

            vy = buff[1] & 0x7F;
            if (buff[1] >> 7)
                vy = -vy;

            vz = buff[2] & 0x7F;
            if (buff[2] >> 7)
                vz = -vz;

            vg = buff[3];
            vq5 = buff[4];

            if (buff[5] >> 7) //Sg
                vg = -vg;
            if ((buff[5] >> 6) & 1) //Sq5
                vq5 = -vq5;

            if (vx || vy || vz || vg)
                widow.movePointWithSpeed(vx, vy, vz, vg, initial_time);
            if (vq5)
                widow.moveServoWithSpeed(4, vq5, initial_time);

            open_close = (buff[5] >> 4) & 0b11;
            switch (open_close)
            {
            case 1:
                widow.moveGrip(0);
                break;
            case 2:
                widow.moveGrip(1);
                break;
            default:
                break;
            }
        }
        else
        {
            switch (options)
            {
            case 1:
                widow.moveRest();
                break;
            case 2:
                widow.moveHome();
                break;
            case 3:
                widow.moveCenter();
                break;
            case 4:
                widow.relaxServos();
                break;
            case 5:
                widow.torqueServos();
                break;
            default:
                break;
            }
        }
    }
}

void printInfo()
{
  
    Serial.print("\nbuff0: ");Serial.println(buff[0]);
    Serial.print("buff1: ");Serial.println(buff[1]);
    Serial.print("buff2: ");Serial.println(buff[2]);
    Serial.print("buff3: ");Serial.println(buff[3]);
    Serial.print("buff4: ");Serial.println(buff[4]);
    Serial.print("buff5: ");Serial.println(buff[5]);
  
}

boolean readData()
{
  byte startByte = 9;
  byte rB;
  while(Serial.available()){
    rB = Serial.read();
    if(rB == startByte)
    {
      for(int i = 0; i < 6; i++)
      {
        while(Serial.available()==0);
        rB = Serial.read();
        if(rB==startByte)
        {
          i = -1;
          continue;
        }else{
          buff[i] = rB;
        }
      }
      return true;
    }
  }
  return false;
}
