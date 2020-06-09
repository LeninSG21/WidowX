/*
MoveWithController.ino - Code to control the WidowX Robot Arm with a controller
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

#include <WidowX.h>

#define NUM_CHARS 6
#define USER_FRIENDLY 0
#define POINT_MOVEMENT 1

WidowX widow = WidowX();

byte buff[NUM_CHARS]; //buffer to save the message
uint8_t open_close, options;
uint8_t moveOption = USER_FRIENDLY;
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

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("...Starting Robotic Arm...");
  delay(300);
  widow.init(0);
  delay(100);

  /*
   * Handshake with the sender.
   * Sends ok\n. Waits until the sender writes ok\n
   * Sends ok\n to indicate that it is ready to receive
  */
  
  Serial.println("ok");
  while (!Serial.available());
  while (Serial.readStringUntil('\n') != "ok")
  {
      while (!Serial.available());
  }
  delay(1000);
  Serial.println("ok");

}

void loop() {
  if(Serial.available())
  {
    Serial.readBytes(buff, NUM_CHARS);
    initial_time = millis();
    options = buff[5] & 0xF; //Get options bits
    
    if (options == 0) //No given option (higher priority)
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
            moveOption = POINT_MOVEMENT;
            break;
        case 7:
            moveOption = USER_FRIENDLY;
            break;
        default:
            break;
        }
    }
    Serial.println("ok");
    
  }

}
