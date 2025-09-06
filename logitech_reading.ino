#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);


void setup()
{
  Serial.begin(115200);


#if !defined(_MIPSEL_)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop()
{
  Usb.Task();

  float Buttons = JoyEvents.bu;
  float lxa =  JoyEvents.lx;        //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
  float lya = JoyEvents.ly;           //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
  float rxa = JoyEvents.rx;          //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
  float rya = JoyEvents.ry;          // map(JoyEvents.ry, 0, 127, 0, 1023); // map(JoyEvents.Rz, 0, 0xFF, 0.f, 255.f);
  //Group initialize
  float blue = Joy.blue;
  float green = Joy.green;
  float red = Joy.red;
  float yellow = Joy.yellow;
  float L1 = Joy.lb;
  float R1 = Joy.rb;
  float gpad = JoyEvents.ht;
  float L2 = Joy.lt;
  float R2 = Joy.rt;
  float back = Joy.bk;
  float start = Joy.st;
  float leftjoy = Joy.jl;
  float rightjoy = Joy.jr;
  lxa = lxa - 128;
  lya = -(lya - 127);
  rxa = rxa - 128;
  rya = rya - 127;


  
  
  Serial.print("Left X: ");
  Serial.print(lxa);
  Serial.print("   Left Y :  ");
  Serial.print(lya);
  Serial.print("  Right X: ");
  Serial.print(rxa);
  Serial.print("   Right Y ");
  Serial.print(rya);
  Serial.print("    ");
  Serial.print("  Blue : ");
  Serial.print(blue);
  Serial.print("  buttons :  ");
  Serial.print(Buttons);
  Serial.print("  yellow :  ");
  Serial.print(yellow);
  Serial.print("  red :  ");
  Serial.print(red);
  Serial.print("  green :  ");
  Serial.print(green);
  Serial.print("    ");
  Serial.print("  LB :  ");
  Serial.print(L1);
  Serial.print("  RB :  ");
  Serial.print(R1);
  Serial.print("  RT :  ");
  Serial.print(R2);
  Serial.print("  gpad :  ");
  Serial.print(gpad);
  Serial.print("  LT :  ");
  Serial.print(L2);
  Serial.print("    ");
  Serial.print("  back :  ");
  Serial.print(back);
  Serial.print("  start :  ");
  Serial.print(start);
  Serial.print("    ");
  Serial.print("  left joy :  ");
  Serial.print(leftjoy);
  Serial.print("  rightjoy :  ");
  Serial.println(rightjoy);

}