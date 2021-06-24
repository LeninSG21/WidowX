#include <WidowX.h>
#include <poses.h>
#include <ax12.h>

WidowX widow = WidowX();

void setup()
{
    Serial.begin(115200);
    Serial.println("...Starting Robotic Arm...");
    delay(1000);
    widow.init(0); //Check voltage, move to rest and do not relax the servos
    delay(2000);
    Serial.println("...Moving Home...");
    widow.moveHome();
    Serial.println("...Moving to Center...");
    widow.moveCenter();
    Serial.println("...Moving to Rest...");
    widow.moveRest();
}

void loop() {}