/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */
#include <XBOXUSB.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXUSB Xbox(&Usb);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);  // <--- Added for Serial1 communication

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial monitor (optional)
#endif

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}

void loop() {
  Usb.Task();
// Serial.println("hii");
  if (Xbox.Xbox360Connected) {
    Serial.println("hello");
    if (Xbox.getButtonPress(LT) || Xbox.getButtonPress(RT)) {
      Serial1.print("LT: ");
      Serial1.print(Xbox.getButtonPress(LT));
      Serial1.print("\tRT: ");
      Serial1.println(Xbox.getButtonPress(RT));
      Xbox.setRumbleOn(Xbox.getButtonPress(LT), Xbox.getButtonPress(RT));
    } else {
      Xbox.setRumbleOn(0, 0);
    }

    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || 
        Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || 
        Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || 
        Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial1.print(F("LeftHatX: "));
        Serial1.print(Xbox.getAnalogHat(LeftHatX));
        Serial1.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
        Serial1.print(F("LeftHatY: "));
        Serial1.print(Xbox.getAnalogHat(LeftHatY));
        Serial1.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial1.print(F("RightHatX: "));
        Serial1.print(Xbox.getAnalogHat(RightHatX));
        Serial1.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        Serial1.print(F("RightHatY: "));
        Serial1.print(Xbox.getAnalogHat(RightHatY));
      }
      Serial1.println();
    }

    // if (Xbox.getButtonClick(UP)) {
    //   Xbox.setLedOn(LED1);
    //   Serial1.println(F("Up"));
    // }
    // if (Xbox.getButtonClick(DOWN)) {
    //   Xbox.setLedOn(LED4);
    //   Serial1.println(F("Down"));
    // }
    // if (Xbox.getButtonClick(LEFT)) {
    //   Xbox.setLedOn(LED3);
    //   Serial1.println(F("Left"));
    // }
    // if (Xbox.getButtonClick(RIGHT)) {
    //   Xbox.setLedOn(LED2);
    //   Serial1.println(F("Right"));
    // }

    if (Xbox.getButtonClick(START)) {
      Xbox.setLedMode(ALTERNATING);
      Serial1.println(F("Start"));
    }
    if (Xbox.getButtonClick(BACK)) {
      Xbox.setLedBlink(ALL);
      Serial1.println(F("Back"));
    }
    if (Xbox.getButtonClick(L3))
      Serial1.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      Serial1.println(F("R3"));

    if (Xbox.getButtonClick(LB))
      Serial1.println(F("LB"));
    if (Xbox.getButtonClick(RB))
      Serial1.println(F("RB"));
    if (Xbox.getButtonClick(XBOX)) {
      Xbox.setLedMode(ROTATING);
      Serial1.println(F("Xbox"));
    }

    if (Xbox.getButtonClick(A))
      Serial1.println(F("A"));
    if (Xbox.getButtonClick(B))
      Serial1.println(F("B"));
    if (Xbox.getButtonClick(X))
      Serial1.println(F("X"));
    if (Xbox.getButtonClick(Y))
      Serial1.println(F("Y"));
  }

  delay(1); // Small delay for stability
}
