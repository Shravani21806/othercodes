// #include <TFT.h>
#include <usbhid.h>
#include <hiduniversal.h>
#include <hidescriptorparser.h>
#include <usbhub.h>
//#include "pgmstrings.h"

#include <XBOXUSB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

class HIDUniversal2 : public HIDUniversal {
public:
  HIDUniversal2(USB *usb)
    : HIDUniversal(usb){};

protected:
  uint8_t OnInitSuccessful();
};

uint8_t HIDUniversal2::OnInitSuccessful() {
  uint8_t rcode;

  HexDumper<USBReadParser, uint16_t, uint16_t> Hex;
  ReportDescParser Rpt;

  if ((rcode = GetReportDescr(0, &Hex)))
    goto FailGetReportDescr1;

  if ((rcode = GetReportDescr(0, &Rpt)))
    goto FailGetReportDescr2;

  return 0;

FailGetReportDescr1:
  USBTRACE("GetReportDescr1:");
  goto Fail;

FailGetReportDescr2:
  USBTRACE("GetReportDescr2:");
  goto Fail;

Fail:
  Serial.println(rcode, HEX);
  Release();
  return rcode;
}
USB Usb;
XBOXUSB Xbox(&Usb);
HIDUniversal2 Hid(&Usb);
UniversalReportParser Uni;


// void (*resetFunc)(void) = 0;
long int rx, ry, lx, ly, up, down, left, right, start, back, yellow, g, blue, red, lb, rb, lt, rt, lj, rj;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Start");
  Usb.Init();
  if (Usb.Init()==-1)
    Serial.println("OSC did not start.");
  // delay(2100);
  if (!Hid.SetReportParser(0, &Uni))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1);
  //sabertooth

}

void loop() {
 
  Usb.Task();
  if (Xbox.Xbox360Connected) 
  { 
    Serial.print("sds");
    lx = Xbox.getAnalogHat(LeftHatX);
    ly = Xbox.getAnalogHat(LeftHatY);
    rx = Xbox.getAnalogHat(RightHatX);
    ry = Xbox.getAnalogHat(RightHatY);
    lx = map(lx, -32768, 32767, -127, 127);
    ly = map(ly, -32768, 32767, -127, 127);
    rx = map(rx, -32768, 32767, -127, 127);
    ry = map(ry, -32768, 32767, -127, 127);
    g = Xbox.getButtonClick(A);
    red = Xbox.getButtonClick(B);
    blue = Xbox.getButtonClick(X);
    yellow = Xbox.getButtonClick(Y);
    lb = Xbox.getButtonClick(L1);
    rb = Xbox.getButtonClick(R1);
    lt = Xbox.getButtonPress(L2);
    rt = Xbox.getButtonPress(R2);
    up = Xbox.getButtonClick(UP);
    down = Xbox.getButtonPress(DOWN);
    left = Xbox.getButtonPress(LEFT);
    right = Xbox.getButtonPress(RIGHT);
    start = Xbox.getButtonClick(START);
    back = Xbox.getButtonClick(BACK);
    lj = Xbox.getButtonPress(L3);
    rj = Xbox.getButtonPress(R3);


    Serial.print(lx);
    Serial.print("   ");
    Serial.print(ly);
    Serial.print(rx);
    Serial.print("   ");
    Serial.print(ry);
    Serial.print("   ");
    Serial.print(g);
    Serial.println("   ");
  }
  // Serial.println(" ");
  delay(5);
}