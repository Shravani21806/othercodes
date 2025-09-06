#include <SPI.h>
#include <RF24.h>
#include <XBOXONE.h>
#include <Usb.h>

#define CE_PIN 23
#define CSN_PIN 22

USB Usb;
XBOXONE Xbox(&Usb);
RF24 radio(CE_PIN, CSN_PIN);

const uint64_t address = 0xF0F0F0F0E1LL;

struct XboxData {
  int LX, LY, RX, RY;
  bool A, B, X, Y, LB, RB, L3, R3, START, BACK, XBOX, UP, DOWN, LEFT, RIGHT;
};

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield not found!");
    while (1)
      ;
  }

  Serial.println("Xbox Controller Ready");
}

void loop() {
  Usb.Task();  // Ensure USB is updated

  if (Xbox.XboxOneConnected) {
    XboxData data;
#define DEADZONE 4000

    int LX_raw = Xbox.getAnalogHat(LeftHatX);
    int LY_raw = Xbox.getAnalogHat(LeftHatY);
    int RX_raw = Xbox.getAnalogHat(RightHatX);
    int RY_raw = Xbox.getAnalogHat(RightHatY);

    // Apply deadzone correction
    data.LX = (abs(LX_raw) > DEADZONE) ? map(LX_raw, -32768, 32767, -255, 255) : 0;
    data.LY = (abs(LY_raw) > DEADZONE) ? map(LY_raw, -32768, 32767, -255, 255) : 0;
    data.RX = (abs(RX_raw) > DEADZONE) ? map(RX_raw, -32768, 32767, -255, 255) : 0;
    data.RY = (abs(RY_raw) > DEADZONE) ? map(RY_raw, -32768, 32767, -255, 255) : 0;

    // Read buttons (use getButtonClick() for single press, getButtonPress() for hold)
    data.A = Xbox.getButtonPress(A);
    data.B = Xbox.getButtonPress(B);
    data.X = Xbox.getButtonClick(X);
    data.Y = Xbox.getButtonClick(Y);
    data.LB = Xbox.getButtonPress(LB);  // LB = Left Bumper
    data.RB = Xbox.getButtonPress(RB);  // RB = Right Bumper
    data.L3 = Xbox.getButtonPress(L3);  // L3 = Left Stick Click
    data.R3 = Xbox.getButtonPress(R3);  // R3 = Right Stick Click
    data.UP = Xbox.getButtonPress(UP);
    data.DOWN = Xbox.getButtonPress(DOWN);
    data.LEFT = Xbox.getButtonPress(LEFT);
    data.RIGHT = Xbox.getButtonPress(RIGHT);
    data.START = Xbox.getButtonClick(START);  // Use getButtonClick for single press detection
    data.BACK = Xbox.getButtonClick(BACK);
    data.XBOX = Xbox.getButtonClick(XBOX);

    // Send Data Continuously
    bool success = radio.write(&data, sizeof(data));

    Serial.print("LX: ");
    Serial.print(data.LX);
    Serial.print(" LY: ");
    Serial.print(data.LY);
    Serial.print(" RX: ");
    Serial.print(data.RX);
    Serial.print(" RY: ");
    Serial.print(data.RY);
    Serial.print(" A: ");
    Serial.print(data.A);
    Serial.print(" B: ");
    Serial.print(data.B);
    Serial.print(" X: ");
    Serial.print(data.X);
    Serial.print(" Y: ");
    Serial.print(data.Y);
    Serial.print(" LB: ");
    Serial.print(data.LB);
    Serial.print(" RB: ");
    Serial.print(data.RB);
    Serial.print(" L3: ");
    Serial.print(data.L3);
    Serial.print(" R3: ");
    Serial.print(data.R3);
    Serial.print(" START: ");
    Serial.print(data.START);
    Serial.print(" BACK: ");
    Serial.print(data.BACK);
    Serial.print(" XBOX: ");
    Serial.print(data.XBOX);
    Serial.print(" UP: ");
    Serial.print(data.UP);
    Serial.print(" DOWN: ");
    Serial.print(data.DOWN);
    Serial.print(" LEFT: ");
    Serial.print(data.LEFT);
    Serial.print(" RIGHT: ");
    Serial.print(data.RIGHT);
    Serial.print(" Status: ");
    Serial.println(success ? "Sent" : "Failed");

    // delayMicroseconds(1000); 
  } else {
    Serial.println("Xbox Controller not connected!");
    // delay(500);  // Prevent unnecessary USB overload
  }
}