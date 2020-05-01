#include <SoftwareSerial.h>

SoftwareSerial BT = SoftwareSerial(10,11);
const byte numChars = 6;
byte buff[numChars];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  BT.begin(9600);
  Serial.println("ok");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    Serial.readBytes(buff,numChars);
    BT.print(Serial.available());
    BT.print("-->");
    for(int i = 0; i<numChars; i++)
    {
      BT.print(buff[i]);
      BT.print('|');
    }
    BT.print("\n");
    Serial.println("ok");
  }
  
}
