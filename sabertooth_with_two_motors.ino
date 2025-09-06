
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 10); 
SabertoothSimplified ST1(SWSerial1); 
// SoftwareSerial SWSerial2(NOT_A_PIN, 3);
// SabertoothSimplified ST2(SWSerial2);
void setup()
{
  SWSerial1.begin(9600);
  SWSerial2.begin(9600);
}

void loop()
{

  ST1.motor(2,-1);
  delay(1000);
  ST1.motor(2,0);

  // ST1.motor(2,127);
  // ST2.motor(1,127);
  // ST2.motor(2,127);
}


