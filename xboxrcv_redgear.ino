#include <XBOXUSB.h>
#include <SPI.h>
#include <usbhub.h>

USB Usb;
XBOXUSB Xbox(&Usb);
int i;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield did not start.");
    while (1); // Halt
  }
  Serial.println("USB Host Shield Started.");
}

void loop() {
  Usb.Task();

  if (Xbox.Xbox360Connected) {
    Serial.println("Xbox 360 Controller connected.");
    Serial.println("A= ");   
    Serial.print( Xbox.getButtonPress(A));
    Serial.print("      B=");   
    Serial.print( Xbox.getButtonPress(B));
    
    // Add more button checks if needed
  } else {
    Serial.println("Waiting for Xbox 360 controller...");
  }

 // delay(200);
}
