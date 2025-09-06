// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.al as the serial port.
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 10);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 3);
SabertoothSimplified ST2(SWSerial2);

void setup()

{
  SWSerial1.begin(9600);
  
  SWSerial2.begin(9600);
}

void loop()
{
  ST2.motor(2,125);
  delay(5);
   ST2.motor(2,0);

  // int power;
  
  // Ramp from -127 to 127 (full reverse to full forward), waiting 20 ms (1/50th of a second) per value.
  // for (power = -127; power <= 127; power ++)
  // {
  //   ST.motor(1, power);
  //   delay(20);
  // }
  
  // // Now go back the way we came.
  // for (power = 127; power >= -127; power --)
  // {
  //   ST.motor(1, power);
  //   delay(20);
  // }
}

