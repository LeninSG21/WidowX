#include <WidowX.h>

WidowX widow = WidowX();
const byte numChars = 6;
byte buff[numChars];
uint8_t open_close, options;
uint8_t moveOption = 0;
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
    delay(1000);
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
    Serial.println("ok");
}

void loop()
{
    if (Serial.available())
    {
        initial_time = millis();
        Serial.readBytes(buff,numChars);
        
        options = buff[5] & 0xF;
        if (options == 0)
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
                
            delay(5);
            if (vq5)
                widow.moveServoWithSpeed(4, vq5, initial_time);
            if (vx || vy || vz || vg)
            {
              if(moveOption)
                widow.movePointWithSpeed(vx, vy, vz, vg, initial_time);
              else
                widow.moveArmWithSpeed(vx, vy, vz, vg, initial_time);
            }
                
            
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
            case 6:
                moveOption = 1;
                break;
            case 7:
                moveOption = 0;
                break;
            default:
                break;
            }
        }
      Serial.println("ok");
    }
}
