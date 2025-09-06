#include "Usb.h"
#include "HIDBoot.h"
#include "xboxEnums.h"

#include <XBOXUSB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXUSB Xbox(&Usb);
// Xbox One data taken from descriptors
#define XBOX_ONE_EP_MAXPKTSIZE 64 // Max size for data via USB

// PID and VID of the different versions of the controller
#define XBOX_VID1 0x2341 // Microsoft Corporation
#define XBOX_ONE_PID1 0x0042 // Microsoft X-Box One pad

/** This class implements support for a Xbox ONE controller connected via USB. */
class XBOXONE : public USBDeviceConfig, public UsbConfigXtracter, public HIDBoot<USB_HID_PROTOCOL_MOUSE> {
public:
    // Constructor for the XBOXONE class.
    XBOXONE(USB *pUsb) : HIDBoot<USB_HID_PROTOCOL_MOUSE>(pUsb, false), pUsb(pUsb), bAddress(0) {}

    // Initialize the Xbox Controller
    virtual uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed) {
        return 0;  // Initialization logic
    }

    // Release the USB device
    virtual uint8_t Release() {
        return 0;  // Release logic
    }

    // Poll the USB Input endpoints and run the state machines
    virtual uint8_t Poll() {
        uint8_t ret = HIDBoot<USB_HID_PROTOCOL_MOUSE>::Poll();  // Poll for input

        if (ret == 0) {
            uint8_t report[64];
            int16_t lx, ly, rx, ry;

            // Get HID report: endpoint, interface, report type, report ID, number of bytes, and buffer
            ret = GetReport(1, 0, 0x01, 0, sizeof(report), report); // 0x01 is for input reports
            if (ret == 0) {
                // Debug: Print the full report to understand its format
                Serial.print("Report received: ");
                for (int i = 0; i < sizeof(report); i++) {
                    Serial.print(report[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

                // Parse joystick data from the HID report (adjust as per actual report format)
                lx = (int16_t)report[2] - 128;  // Left joystick X-axis
                ly = (int16_t)report[3] - 128;  // Left joystick Y-axis
                rx = (int16_t)report[4] - 128;  // Right joystick X-axis
                ry = (int16_t)report[5] - 128;  // Right joystick Y-axis

                // Print joystick values to Serial Monitor
                Serial.print("Left Joystick X: ");
                Serial.print(lx);
                Serial.print("  Left Joystick Y: ");
                Serial.println(ly);

                Serial.print("Right Joystick X: ");
                Serial.print(rx);
                Serial.print("  Right Joystick Y: ");
                Serial.println(ry);
            }
        }
        return ret;
    }

    // Check if the VID/PID matches the controller
    virtual bool VIDPIDOK(uint16_t vid, uint16_t pid) {
        return (vid == XBOX_VID1 && pid == XBOX_ONE_PID1);
    }

    // Test specific VID and PID
    void testVIDPID() {
        if (VIDPIDOK(XBOX_VID1, XBOX_ONE_PID1)) {
            Serial.println("Xbox One Controller detected with VID 2341 and PID 0042.");
        } else {
            Serial.println("Device not recognized as Xbox One Controller.");
        }
    }

private:
    USB *pUsb;
    uint8_t bAddress;
};

// Instantiate the USB object and Xbox controller
USB usb;
XBOXONE xboxOne(&usb);

void setup() {
    Serial.begin(115200);

    // Initialize USB
    if (usb.Init() == -1) {
        Serial.println("USB initialization failed.");
        while (1); // Stop execution if USB initialization fails
    }

    Serial.println("USB initialized.");

    // Test the VID/PID for Xbox One controller
    xboxOne.testVIDPID();
}

void loop() {
    usb.Task(); // Call USB task to handle communication
    xboxOne.Poll(); 
      if (Xbox.Xbox360Connected) {
    if (Xbox.getButtonPress(LT) || Xbox.getButtonPress(RT)) {
      Serial.print("LT: ");
      Serial.print(Xbox.getButtonPress(LT));
      Serial.print("\tRT: ");
      Serial.println(Xbox.getButtonPress(RT));
      Xbox.setRumbleOn(Xbox.getButtonPress(LT), Xbox.getButtonPress(RT));
    } else
      Xbox.setRumbleOn(0, 0);

    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500 || Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.print(F("LeftHatX: "));
        Serial.print(Xbox.getAnalogHat(LeftHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(LeftHatY) > 7500 || Xbox.getAnalogHat(LeftHatY) < -7500) {
        Serial.print(F("LeftHatY: "));
        Serial.print(Xbox.getAnalogHat(LeftHatY));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial.print(F("RightHatX: "));
        Serial.print(Xbox.getAnalogHat(RightHatX));
        Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 || Xbox.getAnalogHat(RightHatY) < -7500) {
        Serial.print(F("RightHatY: "));
        Serial.print(Xbox.getAnalogHat(RightHatY));
      }
      Serial.println();
    }
     if (Xbox.getButtonClick(UP)) {
      Xbox.setLedOn(LED1);
      Serial.println(F("Up"));
    }
    if (Xbox.getButtonClick(DOWN)) {
      Xbox.setLedOn(LED4);
      Serial.println(F("Down"));
    }// Poll the Xbox controller for data
      }}
