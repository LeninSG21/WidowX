#include <WidowX.h>
#include <poses.h>

WidowX widow = WidowX();
float px, py, pz, gamma;
float px_prev, py_prev, pz_prev, gamma_prev;
float point[3] = {0,0,0};
uint8_t buff[6];
uint8_t open_close, options;
uint16_t posq5;

float factor_xy = 43.0/127;
float factor_z = 52.0/170;
float factor_gamma = (M_PI_2)/127;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("...Starting Robotic Arm...");
  delay(300);
  widow.init(0);
  delay(100);
  Serial.println("ok");
  while(!Serial.available());
  while(Serial.readStringUntil('\n')!= "ok")
  {
    while(!Serial.available());
  }

  writePoint();
  
  px_prev = point[0]; py_prev = point[1]; pz_prev = point[2];
  gamma_prev = 0;
  while(Serial.readStringUntil('\n')!= "ok")
  {
    while(!Serial.available());
  }

  delay(1000);
  //widow.moveHome();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    Serial.readBytes(buff, 6);
    options = buff[5] & 0xF;
    if(options == 0 || options > 5)
    {
      //Obtain px
      px = (buff[0] & 0x7F)*factor_xy;
      if(buff[0]>>7)
        px = -px;
        
      //Obtain py
      py = (buff[1] & 0x7F)*factor_xy;
      if(buff[1]>>7)
        py = -py;

      //Obtain pz
      pz = buff[2] * factor_z;
      if(pz>52)
        pz = 52-pz;

      //Obtain gamma
      gamma = (buff[3] & 0x7F)*factor_gamma;
      if(buff[3]>>7)
        gamma = -gamma;

      //Obtain open_close
      open_close = buff[4] >> 6;
      switch(open_close)
      {
        case 1:
          widow.moveGrip(0);//Open grip
          break;
        case 2:
          widow.moveGrip(1);//Close grip
          break;
        default:
          break;
      }
      posq5 = (buff[4] & 0x3F)<<4 | buff[5]>>4;
      widow.setServo2Position(4, posq5);
      
      if(px != px_prev || py != py_prev || pz != pz_prev || gamma != gamma_prev)
      {
        widow.setArmGamma(px,py,pz,gamma);
        
        px_prev = px;
        py_prev = py;
        pz_prev = pz;
        gamma_prev = gamma;
      }
      
    }else{
      /*
      switch(options)
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
      }*/
    }
  }
}

void writePoint()
{
  widow.getPoint(point);
  Serial.println(point[0]);
  Serial.println(point[1]);
  Serial.println(point[2]);
}
