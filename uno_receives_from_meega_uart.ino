#include <Servo.h>

Servo esc1;
Servo esc2;

int Dpad_ref = 0;
int Dpad_ref1 = 0;
int Dpad_ref2 = 0;
int escPin1 = 0;
int escPin2 = 0;

void setup() {
  Serial.begin(115200);  // Initialize USB serial communication for debugging
  Serial2.begin(9600);
  esc1.attach(4);
  esc2.attach(5);  // Attach the ESC to the signal pin
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);  // Initialize Serial2 for UART communication
}

void loop() {
  // Check if data is available on Serial2
  if (Serial2.available()) {
    // Read the incoming string until a newline character
    String receivedData = Serial2.readStringUntil('\n');

    // Parse the received data

    int Dpad = getValue(receivedData, 8);
    Serial.print(" Dpad: ");
    Serial.print(Dpad);
  }
  // bldc
  // bldc calibration
  if (Dpad == 4 && Dpad_ref == 0) {
    Dpad_ref = 1;
  }
  if (Dpad == 0 && Dpad_ref == 1) {
    count1 = 1000;
    count2 = 1000;
    Dpad_ref = 0;
  }
  // bldc increase speed
  if (Dpad == 1 && Dpad_ref1 == 0) {
    Dpad_ref1 = 1;
  }
  if (Dpad == 1 && Dpad_ref1 == 1) {
    count1 = count1 + 50;
    count2 = count2 + 50;
    Dpad_ref1 = 0;
  }
  // bldc decrease speed
  if (Dpad == 2 && Dpad_ref2 == 0) {
    Dpad_ref2 = 1;
  }
  if (Dpad == 0 && Dpad_ref2 == 1) {
    count1 = count1 - 50;
    count2 = count2 - 50;
    Dpad_ref2 = 0;
  }
  // dribbling
  // if (Dpad == 8 && Dpad_ref == 0) {
  //   Dpad_ref = 1;
  // }
  // if (Dpad == 0 && Dpad_ref == 1) {
  //   count1 = 1100;
  //   count2 = 1245;
  //   Dpad_ref = 0;
  // }
  //
  esc1.writeMicroseconds(count1);
  esc2.writeMicroseconds(count2);

  Serial.print("   count1: ");
  Serial.print(count1);
  Serial.print("   count2: ");
  Serial.println(count2);
}

// Function to extract values from the received data string
int getValue(String data, int index) {
  int startIndex = 0;
  int endIndex = data.indexOf(',');
  for (int i = 0; i < index; i++) {
    startIndex = endIndex + 1;
    endIndex = data.indexOf(',', startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
      break;
    }
  }
  return data.substring(startIndex, endIndex).toInt();
}